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

#include <memory>
#include <mrpt_msgs/msg/observation_range_bearing.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stage_ros2/model_wrapper.hpp>
#include <stage_ros2/stage_forward_declarations.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

namespace stage_ros2 {

class FiducialWrapper : public ModelWrapper {
public:
  FiducialWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelFiducial *model,
                  const std::string &ns);

  void publish(std::vector<geometry_msgs::msg::TransformStamped> & transforms,
               const rclcpp::Time &now) override;

protected:
  Stg::ModelFiducial *model_;
  std::string parent_frame_id_;
  std::string frame_id_;
  rclcpp::Publisher<mrpt_msgs::msg::ObservationRangeBearing>::SharedPtr fiducial_pub_;
};

}
