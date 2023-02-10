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

#include <stage_ros2/position_wrapper.hpp>
#include <stage_ros2/utils.hpp>
#include <utility>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/LinearMath/Quaternion.h>
#include <stage.hh>

namespace stage_ros2 {

PositionWrapper::PositionWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelPosition *model,
                                 std::string tf_prefix)
    : model_(model), tf_prefix_(std::move(tf_prefix)),
      odom_pub_(node->create_publisher<nav_msgs::msg::Odometry>("odom", 10)),
      ground_truth_pub_(node->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10)),
      cmd_vel_sub_(node->create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
          std::bind(&PositionWrapper::cmd_vel_callback, this, std::placeholders::_1))) {
}

void PositionWrapper::publish(const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
                              const rclcpp::Time &now) {
  std::string frame_id = "odom";
  std::string child_frame_id = "base_footprint";
  if (!tf_prefix_.empty()) {
    frame_id = tf_prefix_ + "/" + frame_id;
    child_frame_id = tf_prefix_ + "/" + child_frame_id;
  }

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = frame_id;
  odom_msg.header.stamp = now;
  odom_msg.child_frame_id = child_frame_id;
  odom_msg.pose.pose = utils::to_pose_msg(model_->est_pose);
  odom_msg.twist.twist = utils::to_twist_msg(model_->GetVelocity());
  odom_pub_->publish(odom_msg);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom_msg.header;
  transform.child_frame_id = odom_msg.child_frame_id;
  transform.transform.translation.x = odom_msg.pose.pose.position.x;
  transform.transform.translation.y = odom_msg.pose.pose.position.y;
  transform.transform.translation.z = odom_msg.pose.pose.position.z;
  transform.transform.rotation = odom_msg.pose.pose.orientation;
  tf_broadcaster->sendTransform(transform);

  nav_msgs::msg::Odometry ground_truth_msg;
  ground_truth_msg.header.frame_id = frame_id;
  ground_truth_msg.header.stamp = now;
  ground_truth_msg.child_frame_id = child_frame_id;
  ground_truth_msg.pose.pose = utils::to_pose_msg(model_->GetGlobalPose());
  ground_truth_pub_->publish(ground_truth_msg);
}

void PositionWrapper::cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
  model_->SetSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
}

}
