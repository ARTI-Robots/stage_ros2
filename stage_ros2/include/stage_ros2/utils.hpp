#ifndef STAGE_ROS2_UTILS_HPP_
#define STAGE_ROS2_UTILS_HPP_
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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "stage_forward_declarations.hpp"

namespace stage_ros2 {
namespace utils {

rclcpp::Time to_ros_time(uint64_t stage_time);

Stg::Pose to_stage_pose(const geometry_msgs::msg::Pose &pose);

geometry_msgs::msg::Pose to_pose_msg(const Stg::Pose &stage_pose);

geometry_msgs::msg::Transform to_transform_msg(const Stg::Pose &stage_pose);

geometry_msgs::msg::Twist to_twist_msg(const Stg::Velocity &stage_velocity);

Stg::Size to_stage_size(const geometry_msgs::msg::Vector3 &vector);

Stg::Color to_stage_color(const std_msgs::msg::ColorRGBA &color);

Stg::Ancestor *get_parent(const Stg::Model *model);

}
}

#endif //STAGE_ROS2_UTILS_HPP_
