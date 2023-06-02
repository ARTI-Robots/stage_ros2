#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mrpt_msgs/msg/observation_range_bearing.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Converter
{
public:
        explicit Converter(const rclcpp::Node::SharedPtr &node);

        void processObservations(const mrpt_msgs::msg::ObservationRangeBearing::SharedPtr msg_ptr);
private:
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr raw_observation_pub_;
        rclcpp::Subscription<mrpt_msgs::msg::ObservationRangeBearing>::SharedPtr observation_sub_;
};

Converter::Converter(const rclcpp::Node::SharedPtr &node) :
        raw_observation_pub_(node->create_publisher<sensor_msgs::msg::LaserScan>("observations_raw", 10)),
        observation_sub_(node->create_subscription<mrpt_msgs::msg::ObservationRangeBearing>("observations", 10,
         std::bind(&Converter::processObservations, this, std::placeholders::_1)))
{
}

void Converter::processObservations(const mrpt_msgs::msg::ObservationRangeBearing::SharedPtr msg_ptr)
{
        sensor_msgs::msg::LaserScan fake_scan;
        fake_scan.angle_min = -M_PI;
        fake_scan.angle_max = M_PI;
        fake_scan.angle_increment = M_PI/180.;
        fake_scan.header = msg_ptr->header;
        fake_scan.range_min = 0.0;
        fake_scan.range_max = 0.0;
        fake_scan.ranges.resize(static_cast<size_t>(std::ceil((2.*M_PI)/fake_scan.angle_increment)),
                std::numeric_limits<double>::infinity());
        fake_scan.intensities.resize(static_cast<size_t>(std::ceil((2.*M_PI)/fake_scan.angle_increment)),
                std::numeric_limits<double>::infinity());                

        for (const auto &reading : msg_ptr->sensed_data)
        {
                size_t index = static_cast<size_t>(std::round((reading.yaw - fake_scan.angle_min)/fake_scan.angle_increment));
                fake_scan.ranges[index] = reading.range;
                fake_scan.intensities[index] = 1.0;

                if (reading.range > fake_scan.range_max)
                {
                        fake_scan.range_max = reading.range;
                }
        }

        raw_observation_pub_->publish(fake_scan);
}



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("convert_landmarks");
  Converter converter(node);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
