#include "utils.hpp"
#include <tf2/utils.h>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <stage.hh>

namespace utils
{

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
