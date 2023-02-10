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

#include <stage_ros2/utils.hpp>
#include <tf2/utils.h>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <stage.hh>

namespace stage_ros2 {
namespace utils {

rclcpp::Time to_ros_time(Stg::usec_t stage_time) {
  return rclcpp::Time{static_cast<int64_t>(stage_time * 1000), RCL_ROS_TIME};
}

Stg::Pose to_stage_pose(const geometry_msgs::msg::Pose &pose) {
  return {pose.position.x, pose.position.y, pose.position.z, tf2::getYaw(pose.orientation)};
}
geometry_msgs::msg::Pose to_pose_msg(const Stg::Pose &stage_pose) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = stage_pose.x;
  pose.position.y = stage_pose.y;
  pose.position.z = stage_pose.z;
  pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), stage_pose.a));
  return pose;

}
geometry_msgs::msg::Transform to_transform_msg(const Stg::Pose &stage_pose) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = stage_pose.x;
  transform.translation.y = stage_pose.y;
  transform.translation.z = stage_pose.z;
  transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), stage_pose.a));
  return transform;
}

geometry_msgs::msg::Twist to_twist_msg(const Stg::Velocity &stage_velocity) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = stage_velocity.x;
  twist.linear.y = stage_velocity.y;
  twist.linear.z = stage_velocity.z;
  twist.angular.z = stage_velocity.a;
  return twist;
}

Stg::Size to_stage_size(const geometry_msgs::msg::Vector3 &vector) {
  return {vector.x, vector.y, vector.z};
}

Stg::Color to_stage_color(const std_msgs::msg::ColorRGBA &color) {
  // This also fixes the default value for alpha to prevent invisible models:
  return Stg::Color{color.r, color.g, color.b, (color.a != 0.0) ? color.a : 1.0};
}

Stg::Ancestor *get_parent(const Stg::Model *model) {
  Stg::Model *const parent = model->Parent();
  if (parent) {
    return parent;
  }
  return model->GetWorld();
}

}
}
