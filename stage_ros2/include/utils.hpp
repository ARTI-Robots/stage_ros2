#ifndef STAGE_ROS2_UTILS_HPP_
#define STAGE_ROS2_UTILS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "stage_forward_declarations.hpp"

namespace utils {

rclcpp::Time to_ros_time(uint64_t stage_time);

Stg::Pose to_stage_pose(const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::Pose to_pose_msg(const Stg::Pose& stage_pose);

geometry_msgs::msg::Transform to_transform_msg(const Stg::Pose& stage_pose);

Stg::Size to_stage_size(const geometry_msgs::msg::Vector3& vector);

Stg::Color to_stage_color(const std_msgs::msg::ColorRGBA& color);

Stg::Ancestor *get_parent(const Stg::Model *model);

}

#endif //STAGE_ROS2_UTILS_HPP_
