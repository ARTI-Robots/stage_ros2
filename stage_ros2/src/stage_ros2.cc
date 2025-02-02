/*
 *  stage_ros2: ROS 2 node wrapping the Stage simulator.
 *
 *  Copyright (C) 2023 ARTI - Autonomous Robot Technology GmbH
 *  Copyright (C) 2020 ymd-stella
 *  Copyright (C) 2001-2009 Richard Vaughan, Brian Gerkey, Andrew
 *  Howard, Toby Collett, Reed Hedges, Alex Couture-Beil, Jeremy
 *  Asher, Pooya Karimian
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <stage.hh>
#include <stage_ros2/model_server.hpp>
#include <stage_ros2/position_wrapper.hpp>
#include <stage_ros2/utils.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace stage_ros2 {

class StageWrapper {
public:
    bool init(int argc, char** argv) {
        rclcpp::init(argc, argv);
        rclcpp::uninstall_signal_handlers();
        Stg::Init(&argc, &argv);

        node_ = rclcpp::Node::make_shared("stage_ros2");
        private_ns_node_ = node_->create_sub_node(node_->get_name());
        clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("clock", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
        const auto gui = node_->declare_parameter("gui", true);
        const auto world_file = node_->declare_parameter<std::string>("world");

        executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
        executor_->add_node(node_);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

        world_ = gui ? std::make_shared<Stg::WorldGui>(600, 400, "stage_ros2")
                     : std::make_shared<Stg::World>();
        if (!world_->Load(world_file)) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                "failed to load world file '" << world_file << "'");
            return false;
        }

        model_server_ = std::make_shared<ModelServer>(private_ns_node_, world_);
        world_->AddUpdateCallback([](Stg::World*, void *user){
            static_cast<StageWrapper*>(user)->world_update_callback();
            // Return false to indicate that we want to be called again (an odd convention, but
            // that's the way that Stage works):
            return 0;
        }, this);
        world_->ForEachDescendant([](Stg::Model* mod, void* user){
            static_cast<StageWrapper*>(user)->search_and_init_robot(mod);
            return 0;
        }, this);
        world_->Start();
        ros2_thread_ = std::make_shared<std::thread>([this](){ executor_->spin(); });
        return true;
    }

    ~StageWrapper() {
        rclcpp::shutdown();
        ros2_thread_->join();
    }

private:
    std::shared_ptr<std::thread> ros2_thread_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr private_ns_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    std::shared_ptr<Stg::World> world_;
    std::shared_ptr<ModelServer> model_server_;
    std::unordered_map<Stg::Model*, std::shared_ptr<PositionWrapper>> robots_;

    void search_and_init_robot(Stg::Model* model) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "[search robots] token: " << model->Token() << ", type: " << model->GetModelType());
        if (const auto position_model = dynamic_cast<Stg::ModelPosition *>(model)) {
            robots_[model] = std::make_shared<PositionWrapper>(private_ns_node_, position_model, "");
        } else {
            const auto parent_robot_it = robots_.find(model->Parent());
            if (parent_robot_it != robots_.end()) {
                parent_robot_it->second->wrap_sensor(model);
            }
        }
    }

    void world_update_callback() {
        const rclcpp::Time now = utils::to_ros_time(world_->SimTimeNow());

        rosgraph_msgs::msg::Clock clock;
        clock.clock = now;
        clock_pub_->publish(clock);

        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        for (const auto& pair : robots_) {
            pair.second->publish(transforms, now);
        }

        tf_broadcaster_->sendTransform(transforms);
    }
};

}

int main(int argc, char** argv) {
    stage_ros2::StageWrapper stage_wrapper;
    if (!stage_wrapper.init(argc, argv)) {
        return 1;
    }
    Stg::World::Run();
    return 0;
}
