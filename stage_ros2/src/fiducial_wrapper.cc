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

#include <stage_ros2/fiducial_wrapper.hpp>
#include <mrpt_msgs/msg/single_range_bearing_observation.hpp>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/LinearMath/Quaternion.h>
#include <stage.hh>

namespace stage_ros2 {

FiducialWrapper::FiducialWrapper(const rclcpp::Node::SharedPtr &node, Stg::ModelFiducial *model,
                                 const std::string &ns)
    : ModelWrapper(model, ns), model_(model), parent_frame_id_(ns + "base_link"),
      frame_id_(private_ns_ + "base_fiducial"),
      fiducial_pub_(node->create_publisher<mrpt_msgs::msg::ObservationRangeBearing>(
          private_ns_ + "observations", 10)) {
}

void FiducialWrapper::publish(std::vector<geometry_msgs::msg::TransformStamped> & transforms,
                              const rclcpp::Time &now) {
  const auto &fiducials = model_->GetFiducials();
  mrpt_msgs::msg::ObservationRangeBearing msg;
  msg.header.stamp = now;
  msg.header.frame_id = frame_id_;
  msg.min_sensor_distance = model_->min_range;
  msg.max_sensor_distance = model_->max_range_anon;
  msg.sensed_data.reserve(fiducials.size());
  for (const Stg::ModelFiducial::Fiducial &fiducial : fiducials) {
    mrpt_msgs::msg::SingleRangeBearingObservation observation;
    observation.range = fiducial.range;
    observation.yaw = fiducial.bearing;
    observation.pitch = 0.;
    observation.id = fiducial.id;
    msg.sensed_data.emplace_back(observation);
  }

  fiducial_pub_->publish(msg);

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
  transforms.push_back(transform);
}

}
