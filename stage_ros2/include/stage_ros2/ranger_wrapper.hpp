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

#include <stage.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/LinearMath/Quaternion.h>

namespace stage_ros2 {

class RangerWrapper
{
public:
    RangerWrapper(const rclcpp::Node::SharedPtr& node, Stg::ModelRanger* model, const std::string& name, const std::string& tf_prefix)
      :model_(model)
    {
        parent_frame_id_ = "base_link";
        if (tf_prefix.size() > 0) {
            parent_frame_id_ = tf_prefix + "/" + parent_frame_id_;
        }
        frame_id_ = name + "/base_scan";
        if (tf_prefix.size() > 0) {
            frame_id_ = tf_prefix + "/" + frame_id_;
        }
        laser_scan_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(name, 10);
        is_sonar = model_->GetSensors()[0].sample_count == 1;
        if (is_sonar) {
            RCLCPP_WARN_STREAM(node->get_logger(), "sonar is not supported");
        }
    }

    void publish(const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster, const rclcpp::Time& now) {
        if (is_sonar) {
            return;
        }
        auto sensor = model_->GetSensors()[0];
        sensor_msgs::msg::LaserScan msg;
        msg.angle_max = sensor.fov / 2.0;
        msg.angle_min = -sensor.fov / 2.0;
        msg.angle_increment = sensor.fov / (double)(sensor.sample_count - 1);
        msg.range_max = sensor.range.max;
        msg.range_min = sensor.range.min;
        msg.ranges.resize(sensor.ranges.size());
        msg.intensities.resize(sensor.intensities.size());
        for(unsigned int i = 0; i < sensor.ranges.size(); i++)
        {
            msg.ranges[i] = sensor.ranges[i];
            msg.intensities[i] = sensor.intensities[i];
        }
        msg.header.frame_id = frame_id_;
        msg.header.stamp = now;
        laser_scan_pub_->publish(msg);

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
        tf_broadcaster->sendTransform(transform);
    }
    Stg::ModelRanger* model_;
    std::string parent_frame_id_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    bool is_sonar;
};

}
