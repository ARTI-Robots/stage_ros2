#ifndef STAGE_ROS2_MODEL_SERVER_HPP_
#define STAGE_ROS2_MODEL_SERVER_HPP_
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
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stage_ros2_itfs/action/move_model.hpp>
#include <stage_ros2_itfs/srv/create_model.hpp>
#include <stage_ros2_itfs/srv/remove_model.hpp>
#include <stage_ros2_itfs/srv/remove_models.hpp>
#include "stage_forward_declarations.hpp"

namespace stage_ros2 {

class ModelServer {
public:
  ModelServer(rclcpp::Node::SharedPtr node, std::shared_ptr<Stg::World> world);

protected:
  using CreateModel = stage_ros2_itfs::srv::CreateModel;
  using RemoveModel = stage_ros2_itfs::srv::RemoveModel;
  using RemoveModelStatus = RemoveModel::Response::_status_type;
  using RemoveModels = stage_ros2_itfs::srv::RemoveModels;
  using MoveModel = stage_ros2_itfs::action::MoveModel;
  using MoveModelGoalHandleSharedPtr = std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveModel>>;
  using MoveModelGoalHandleQueue = std::queue<MoveModelGoalHandleSharedPtr>;
  using MoveModelGoalHandleQueueMap = std::map<std::string, MoveModelGoalHandleQueue>;

  void create_model(const CreateModel::Request::ConstSharedPtr &request,
                    const CreateModel::Response::SharedPtr &response);
  void remove_model(const RemoveModel::Request::ConstSharedPtr &request,
                    const RemoveModel::Response::SharedPtr &response);
  void remove_models(const RemoveModels::Request::ConstSharedPtr &request,
                     const RemoveModels::Response::SharedPtr &response);
  void move_model(const MoveModelGoalHandleSharedPtr &goal_handle);

  RemoveModelStatus do_remove_model(const std::string &id) const;
  bool validate_trajectory(const MoveModel::Goal &move_model_goal) const;
  static rclcpp::Time get_end_time(const MoveModel::Goal &move_model_goal);
  void enqueue_control_callback();
  void control();
  void control_model(Stg::Model *model, MoveModelGoalHandleQueue& goal_handle_queue) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Stg::World> world_;

  rclcpp::Service<CreateModel>::SharedPtr create_model_service_;
  rclcpp::Service<RemoveModel>::SharedPtr remove_model_service_;
  rclcpp::Service<RemoveModels>::SharedPtr remove_models_service_;
  rclcpp_action::Server<MoveModel>::SharedPtr move_model_action_server_;
  MoveModelGoalHandleQueueMap move_action_goal_handle_queues_by_model_id_;
};

}

#endif //STAGE_ROS2_MODEL_SERVER_HPP_
