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

#include <stage_ros2/ranger_wrapper.hpp>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/LinearMath/Quaternion.h>
#include <stage.hh>

namespace stage_ros2 {

RangerWrapper::RangerWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelRanger *model,
                             const std::string &ns)
    : ModelWrapper(model, ns), model_(model), parent_frame_id_(ns + "base_link"),
      frame_id_(private_ns_ + "base_scan") {
  laser_scan_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(private_ns_ + "scan", 10);
  is_sonar_ = model_->GetSensors().at(0).sample_count == 1;
  if (is_sonar_) {
    RCLCPP_WARN_STREAM(node->get_logger(), "sonar is not supported");
  }
}

void RangerWrapper::publish(const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
                            const rclcpp::Time &now) {
  if (is_sonar_) {
    return;
  }
  const auto &sensor = model_->GetSensors().at(0);
  sensor_msgs::msg::LaserScan msg;
  msg.angle_max = sensor.fov / 2.0;
  msg.angle_min = -sensor.fov / 2.0;
  msg.angle_increment = sensor.fov / (double)(sensor.sample_count - 1);
  msg.range_max = sensor.range.max;
  msg.range_min = sensor.range.min;
  msg.ranges.assign(sensor.ranges.begin(), sensor.ranges.end());
  msg.intensities.assign(sensor.intensities.begin(), sensor.intensities.end());
  msg.header.frame_id = frame_id_;
  msg.header.stamp = now;
  laser_scan_pub_->publish(msg);

  Stg::Pose p = model_->GetPose();
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p.a);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent_frame_id_;
  transform.header.stamp = now;
  transform.child_frame_id = frame_id_;
  transform.transform.translation.x = p.x;
  transform.transform.translation.y = p.y;
  transform.transform.translation.z = p.z;
  transform.transform.rotation = toMsg(q);
  tf_broadcaster->sendTransform(transform);
}

}
