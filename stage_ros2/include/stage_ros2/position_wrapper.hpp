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
#include <nav_msgs/msg/odometry.hpp>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace stage_ros2 {

class PositionWrapper
{
public:
    PositionWrapper(const rclcpp::Node::SharedPtr& node, Stg::ModelPosition* model, const std::string& tf_prefix)
      :model_(model)
    {
        tf_prefix_ = tf_prefix;
        odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        ground_truth_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10);
    }

    void set_speed(double vx, double vy, double avz) {
        model_->SetSpeed(vx, vy, avz);
    }

    void publish(const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster, const rclcpp::Time& now) {
        std::string frame_id = "odom";
        std::string child_frame_id = "base_footprint";
        if (tf_prefix_.size() > 0) {
            frame_id = tf_prefix_ + "/" + frame_id;
            child_frame_id = tf_prefix_ + "/" + child_frame_id;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.pose.pose.position.x = model_->est_pose.x;
        odom_msg.pose.pose.position.y = model_->est_pose.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, model_->est_pose.a);
        odom_msg.pose.pose.orientation = toMsg(q);
        Stg::Velocity v = model_->GetVelocity();
        odom_msg.twist.twist.linear.x = v.x;
        odom_msg.twist.twist.linear.y = v.y;
        odom_msg.twist.twist.angular.z = v.a;
        odom_msg.header.frame_id = frame_id;
        odom_msg.child_frame_id = child_frame_id;
        odom_msg.header.stamp = now;
        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped transform;
        transform.header = odom_msg.header;
        transform.child_frame_id = odom_msg.child_frame_id;
        transform.transform.translation.x = odom_msg.pose.pose.position.x;
        transform.transform.translation.y = odom_msg.pose.pose.position.y;
        transform.transform.translation.z = odom_msg.pose.pose.position.z;
        transform.transform.rotation.x = odom_msg.pose.pose.orientation.x;
        transform.transform.rotation.y = odom_msg.pose.pose.orientation.y;
        transform.transform.rotation.z = odom_msg.pose.pose.orientation.z;
        transform.transform.rotation.w = odom_msg.pose.pose.orientation.w;
        tf_broadcaster->sendTransform(transform);

        auto global_pose = model_->GetGlobalPose();
        nav_msgs::msg::Odometry ground_truth_msg;
        ground_truth_msg.pose.pose.position.x = global_pose.x;
        ground_truth_msg.pose.pose.position.y = global_pose.y;
        tf2::Quaternion q_global_pose;
        q_global_pose.setRPY(0.0, 0.0, global_pose.a);
        ground_truth_msg.pose.pose.orientation = toMsg(q_global_pose);
        ground_truth_msg.header.frame_id = frame_id;
        ground_truth_msg.child_frame_id = child_frame_id;
        ground_truth_msg.header.stamp = now;
        ground_truth_pub_->publish(ground_truth_msg);
    }

    Stg::ModelPosition* model_;
    std::string tf_prefix_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
};

}
