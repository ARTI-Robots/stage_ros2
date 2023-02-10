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

#include <stage_ros2/robot_wrapper.hpp>
#include <algorithm>
#include <stage_ros2/camera_wrapper.hpp>
#include <stage_ros2/position_wrapper.hpp>
#include <stage_ros2/ranger_wrapper.hpp>
#include <stage.hh>

namespace stage_ros2 {

RobotWrapper::RobotWrapper(const rclcpp::Node::SharedPtr &node, const std::string &name)
    : node_(node->create_sub_node(node->get_name())->create_sub_node(name)),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)), tf_prefix_(name) {
}

void RobotWrapper::publish(const rclcpp::Time &now) {
  geometry_msgs::msg::TransformStamped transform;
  std::string frame_id = "base_footprint";
  std::string child_frame_id = "base_link";
  if (!tf_prefix_.empty()) {
    frame_id = tf_prefix_ + "/" + frame_id;
    child_frame_id = tf_prefix_ + "/" + child_frame_id;
  }
  transform.header.frame_id = frame_id;
  transform.header.stamp = now;
  transform.child_frame_id = child_frame_id;
  tf_broadcaster_->sendTransform(transform);

  position_->publish(tf_broadcaster_, now);
  for (auto &ranger : rangers_) {
    ranger->publish(tf_broadcaster_, now);
  }
  for (auto &camera : cameras_) {
    camera->publish(tf_broadcaster_, now);
  }
}

void RobotWrapper::wrap(Stg::Model *mod) {
  if (mod->GetModelType() == "position") {
    mod->Subscribe();
    position_ = std::make_shared<PositionWrapper>(node_, static_cast<Stg::ModelPosition *>(mod),
                                                  tf_prefix_);
  } else {
    // token -> model_name
    std::string model_name = mod->Token();
    auto pos_dot = model_name.find('.');
    if (pos_dot != std::string::npos) {
      model_name = model_name.substr(pos_dot + 1);
    }

    std::replace(model_name.begin(), model_name.end(), ':', '_');

    if (mod->GetModelType() == "ranger") {
      mod->Subscribe();
      rangers_.push_back(std::make_shared<RangerWrapper>(node_,
                                                         static_cast<Stg::ModelRanger *>(mod),
                                                         model_name, tf_prefix_));
    } else if (mod->GetModelType() == "camera") {
      mod->Subscribe();
      cameras_.push_back(std::make_shared<CameraWrapper>(node_,
                                                         static_cast<Stg::ModelCamera *>(mod),
                                                         model_name, tf_prefix_));
    } else if (mod->GetModelType() != "model") {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "sensor type '" << mod->GetModelType() << "' is not supported");
    }
  }
}

}
