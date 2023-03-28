#pragma once
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

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stage_ros2/model_wrapper.hpp>
#include <stage_ros2/stage_forward_declarations.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

namespace stage_ros2 {

class PositionWrapper : public ModelWrapper {
public:
  PositionWrapper(rclcpp::Node::SharedPtr node, Stg::ModelPosition *model, const std::string &ns);

  void wrap_sensor(Stg::Model *model);

  void publish(std::vector<geometry_msgs::msg::TransformStamped> & transforms,
               const rclcpp::Time &now) override;

protected:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);

  rclcpp::Node::SharedPtr node_;
  Stg::ModelPosition *model_;
  std::vector<std::shared_ptr<ModelWrapper>> sensors_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}
