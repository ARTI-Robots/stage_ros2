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
#include <chrono>
#include <stage_ros2/camera_wrapper.hpp>
#include <stage_ros2/fiducial_wrapper.hpp>
#include <stage_ros2/ranger_wrapper.hpp>
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

using namespace std::chrono_literals;

namespace stage_ros2 {

PositionWrapper::PositionWrapper(rclcpp::Node::SharedPtr node, Stg::ModelPosition *model,
                                 const std::string &ns)
    : ModelWrapper(model, ns), node_(std::move(node)), model_(model),
      odom_pub_(node_->create_publisher<nav_msgs::msg::Odometry>(private_ns_ + "odom", 10)),
      ground_truth_pub_(node_->create_publisher<nav_msgs::msg::Odometry>(
          private_ns_ + "ground_truth", 10)),
      cmd_vel_sub_(node_->create_subscription<geometry_msgs::msg::Twist>(
          private_ns_ + "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
          std::bind(&PositionWrapper::cmd_vel_callback, this, std::placeholders::_1))) {
}

void PositionWrapper::wrap_sensor(Stg::Model *model) {
  if (const auto ranger_model = dynamic_cast<Stg::ModelRanger *>(model)) {
    sensors_.push_back(std::make_shared<RangerWrapper>(node_, ranger_model, private_ns_));
  } else if (const auto camera_model = dynamic_cast<Stg::ModelCamera *>(model)) {
    sensors_.push_back(std::make_shared<CameraWrapper>(node_, camera_model, private_ns_));
  } else if (const auto fiducial_model = dynamic_cast<Stg::ModelFiducial *>(model)) {
    sensors_.push_back(std::make_shared<FiducialWrapper>(node_, fiducial_model, private_ns_));
  } else if (model->GetModelType() != "model") {
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "sensor type '" << model->GetModelType() << "' is not supported");
  }
}

void PositionWrapper::publish(std::vector<geometry_msgs::msg::TransformStamped> & transforms,
                              const rclcpp::Time &now) {
  if (command_timeout_ < now) {
    model_->SetSpeed(0, 0, 0);
  }

  nav_msgs::msg::Odometry odom_msg;
  {
    odom_msg.header.frame_id = private_ns_ + "odom";
    odom_msg.header.stamp = now;
    odom_msg.child_frame_id = private_ns_ + "base_footprint";
    odom_msg.pose.pose = utils::to_pose_msg(model_->est_pose);
    odom_msg.twist.twist = utils::to_twist_msg(model_->GetVelocity());
    odom_pub_->publish(odom_msg);
  }

  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = private_ns_ + "base_footprint";
    transform.header.stamp = now;
    transform.child_frame_id = private_ns_ + "base_link";
    transforms.push_back(transform);
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom_msg.header;
  transform.child_frame_id = odom_msg.child_frame_id;
  transform.transform.translation.x = odom_msg.pose.pose.position.x;
  transform.transform.translation.y = odom_msg.pose.pose.position.y;
  transform.transform.translation.z = odom_msg.pose.pose.position.z;
  transform.transform.rotation = odom_msg.pose.pose.orientation;
  transforms.push_back(transform);

  nav_msgs::msg::Odometry ground_truth_msg;
  ground_truth_msg.header = odom_msg.header;
  ground_truth_msg.child_frame_id = odom_msg.child_frame_id;
  ground_truth_msg.pose.pose = utils::to_pose_msg(model_->GetGlobalPose());
  ground_truth_pub_->publish(ground_truth_msg);

  for (const auto &sensor : sensors_) {
    sensor->publish(transforms, now);
  }
}

void PositionWrapper::cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
  command_timeout_ = utils::to_ros_time(model_->GetWorld()->SimTimeNow()) + 1s;
  model_->SetSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
}

}
