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

#include <limits>
#include <stage.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// see CMakeLists.txt
#ifdef USE_LEGACY_MSGS_INCLUDE
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace stage_ros2 {

class CameraWrapper
{
public:
    CameraWrapper(const rclcpp::Node::SharedPtr& node, Stg::ModelCamera* model, const std::string& name, const std::string& tf_prefix)
      :model_(model)
    {
        parent_frame_id_ = "base_link";
        if (tf_prefix.size() > 0) {
            parent_frame_id_ = tf_prefix + "/" + parent_frame_id_;
        }
        frame_id_ = name + "/base_camera";
        if (tf_prefix.size() > 0) {
            frame_id_ = tf_prefix + "/" + frame_id_;
        }
        camera_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(name + "/camera_info", 10);
        image_pub_ = node->create_publisher<sensor_msgs::msg::Image>(name + "/image_raw", 10);
        depth_pub_ = node->create_publisher<sensor_msgs::msg::Image>(name + "/depth", 10);
        // TODO: verify that get_parameter_or really works
        node->get_parameter_or("is_depth_canonical", is_depth_canonical_, true);
    }

    void publish(const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster, const rclcpp::Time& now) {
        publish_image(now);
        publish_depth(now);
        publich_camera_info(now);
        publish_tf(tf_broadcaster, now);
    }

    void publish_image(const rclcpp::Time& now) {
        auto frame_color = model_->FrameColor();
        sensor_msgs::msg::Image image_msg;
        image_msg.height = model_->getHeight();
        image_msg.width = model_->getWidth();
        image_msg.encoding = "rgba8";
        image_msg.step = image_msg.width * 4;
        image_msg.data.resize(image_msg.step * image_msg.height);
        auto src = &(frame_color[(image_msg.height - 1) * image_msg.step]);
        auto dst = &(image_msg.data[0]);
        for (unsigned int y = 0; y < image_msg.height; ++y, src -= image_msg.step, dst += image_msg.step) {
            memcpy(dst, src, image_msg.step);
        }
        image_msg.header.frame_id = frame_id_;
        image_msg.header.stamp = now;
        image_pub_->publish(image_msg);
    }

    void publish_depth(const rclcpp::Time& now) {
        auto frame_depth = model_->FrameDepth();
        sensor_msgs::msg::Image depth_msg;
        depth_msg.height = model_->getHeight();
        depth_msg.width = model_->getWidth();
        int cell_size;
        if (is_depth_canonical_) {
            depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            cell_size = sizeof(float);
        }
        else{
            depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            cell_size = sizeof(uint16_t);
        }
        depth_msg.step = depth_msg.width * cell_size;
        depth_msg.data.resize(depth_msg.step * depth_msg.height);

        if (is_depth_canonical_){
            auto depth_min = static_cast<float>(model_->getCamera().nearClip());
            auto depth_max = static_cast<float>(model_->getCamera().farClip());
            auto dst = reinterpret_cast<float*>(&(depth_msg.data)[0]);
            auto src = &(frame_depth[(depth_msg.height - 1) * depth_msg.width]);
            for (unsigned int y = 0; y < depth_msg.height; ++y, src -= depth_msg.width, dst += depth_msg.width) {
                for (size_t x = 0; x < depth_msg.width; ++x){
                    auto depth = src[x];
                    if (depth <= depth_min) {
                        depth = -std::numeric_limits<float>::infinity();
                    }
                    else if (depth >= depth_max) {
                        depth = std::numeric_limits<float>::infinity();
                    }
                    dst[x] = depth;
                }
            }
        }
        else{
            auto depth_min = static_cast<uint16_t>(model_->getCamera().nearClip() * 1000.f);
            auto depth_max = static_cast<uint16_t>(model_->getCamera().farClip() * 1000.f);
            auto dst = reinterpret_cast<uint16_t*>(&(depth_msg.data[0]));
            auto src = &(frame_depth[(depth_msg.height - 1) * depth_msg.width]);
            for (unsigned int y = 0; y < depth_msg.height; ++y, src -= depth_msg.width, dst += depth_msg.width) {
                for (size_t x = 0; x < depth_msg.width; ++x){
                    auto depth = static_cast<uint16_t>(src[x] * 1000.f);
                    if (depth <= depth_min || depth >= depth_max) {
                        depth = 0;
                    }
                    dst[x] = depth;
                }
            }
        }
        depth_msg.header.frame_id = frame_id_;
        depth_msg.header.stamp = now;
        depth_pub_->publish(depth_msg);
    }

    void publich_camera_info(const rclcpp::Time& now) {
        sensor_msgs::msg::CameraInfo camera_info_msg;
        camera_info_msg.header.frame_id = frame_id_;
        camera_info_msg.header.stamp = now;
        camera_info_msg.height = model_->getHeight();
        camera_info_msg.width = model_->getWidth();

        double fx,fy,cx,cy;
        cx = camera_info_msg.width / 2.0;
        cy = camera_info_msg.height / 2.0;
        double fovh = model_->getCamera().horizFov() * M_PI / 180.0;
        double fovv = model_->getCamera().vertFov() * M_PI / 180.0;
        fx = model_->getWidth() / (2 * std::tan(fovh / 2));
        fy = model_->getHeight() / (2 * std::tan(fovv / 2));
        camera_info_msg.d.resize(4, 0.0);

        camera_info_msg.k[0] = fx;
        camera_info_msg.k[2] = cx;
        camera_info_msg.k[4] = fy;
        camera_info_msg.k[5] = cy;
        camera_info_msg.k[8] = 1.0;

        camera_info_msg.r[0] = 1.0;
        camera_info_msg.r[4] = 1.0;
        camera_info_msg.r[8] = 1.0;

        camera_info_msg.p[0] = fx;
        camera_info_msg.p[2] = cx;
        camera_info_msg.p[5] = fy;
        camera_info_msg.p[6] = cy;
        camera_info_msg.p[10] = 1.0;

        camera_info_pub_->publish(camera_info_msg);
    }

    void publish_tf(const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster, const rclcpp::Time& now) {
        Stg::Pose p = model_->GetPose();
        tf2::Quaternion q;
        // We assume that yaw_offset is 0.
        q.setRPY(0.0, Stg::dtor(90.0 - model_->getCamera().pitch()), p.a);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = parent_frame_id_;
        transform.header.stamp = now;
        transform.child_frame_id = frame_id_;
        transform.transform.translation.x = p.x;
        transform.transform.translation.y = p.y;
        transform.transform.translation.z = p.z;
        transform.transform.rotation = tf2::toMsg(q);
        tf_broadcaster->sendTransform(transform);
    }

    Stg::ModelCamera* model_;
    std::string parent_frame_id_;
    std::string frame_id_;
    bool is_depth_canonical_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

}
