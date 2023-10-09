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

#include <stage_ros2/model_server.hpp>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stage_ros2/utils.hpp>
#include <utility>
#include <sstream>
#include <stage.hh>
#undef Success
#include <unsupported/Eigen/Splines>

namespace stage_ros2 {

namespace phs = std::placeholders;

ModelServer::ModelServer(rclcpp::Node::SharedPtr node, std::shared_ptr<Stg::World> world)
    : node_(std::move(node)), world_(std::move(world)),
      create_model_service_(node_->create_service<stage_ros2_itfs::srv::CreateModel>(
          "create_model", std::bind(&ModelServer::create_model, this, phs::_1, phs::_2))),
      remove_model_service_(node_->create_service<stage_ros2_itfs::srv::RemoveModel>(
          "remove_model", std::bind(&ModelServer::remove_model, this, phs::_1, phs::_2))),
      remove_models_service_(node_->create_service<stage_ros2_itfs::srv::RemoveModels>(
          "remove_models", std::bind(&ModelServer::remove_models, this, phs::_1, phs::_2))),
      move_model_action_server_(rclcpp_action::create_server<stage_ros2_itfs::action::MoveModel>(
          // ActionServer doesn't take sub-namespace into account correctly:
          node_, node_->get_effective_namespace() + "/move_model",
          [](...) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
          [](...) { return rclcpp_action::CancelResponse::ACCEPT; },
          std::bind(&ModelServer::move_model, this, phs::_1), make_action_server_options())) {
  enqueue_control_callback();
}

rcl_action_server_options_t ModelServer::make_action_server_options() {
  rcl_action_server_options_t action_server_options = rcl_action_server_get_default_options();
  action_server_options.goal_service_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  action_server_options.cancel_service_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  action_server_options.result_service_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  action_server_options.feedback_topic_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  action_server_options.status_topic_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  return action_server_options;
}

void ModelServer::create_model(
    const CreateModel::Request::ConstSharedPtr &request,
    const CreateModel::Response::SharedPtr &response) {
  if (!request->type.empty()) {
    // Check whether model type exists. We do this because Stage will exit() if we call
    // CreateModel with a nonexistent type:
    const auto model_type_it = Stg::Model::name_map.find(request->type);
    if (model_type_it == Stg::Model::name_map.end() || model_type_it->second == nullptr) {
      RCLCPP_ERROR(node_->get_logger(),
                   "tried to create model with invalid type '%s'", request->type.c_str());
      response->status = stage_ros2_itfs::srv::CreateModel::Response::STATUS_INVALID_TYPE;
      return;
    }
  }

  // Create a new model. It automatically adds itself to the given world:
  Stg::Model *const model = world_->CreateModel(nullptr, !request->type.empty() ? request->type
                                                                                : "model");
  if (model == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Stage refused to create model");
    response->status = stage_ros2_itfs::srv::CreateModel::Response::STATUS_STAGE_ERROR;
    return;
  }

  if (!request->id.empty()) {
    model->SetToken(request->id);
  }
  model->SetPose(utils::to_stage_pose(request->pose));
  model->SetGeom(Stg::Geom{Stg::Pose{}, utils::to_stage_size(request->size)});
  model->SetColor(utils::to_stage_color(request->color));

  RCLCPP_INFO(node_->get_logger(), "created model '%s'", request->id.c_str());
  response->id = model->TokenStr();
}

void ModelServer::remove_model(const RemoveModel::Request::ConstSharedPtr &request,
                               const RemoveModel::Response::SharedPtr &response) {
  response->status = do_remove_model(request->id);
}

void ModelServer::remove_models(const RemoveModels::Request::ConstSharedPtr &request,
                                const RemoveModels::Response::SharedPtr &response) {
  response->statuses.reserve(request->ids.size());
  for (const std::string &id : request->ids) {
    response->statuses.emplace_back(do_remove_model(id));
  }
}

void ModelServer::move_model(const MoveModelGoalHandleSharedPtr &goal_handle) {
  const auto goal = goal_handle->get_goal();
  Stg::Model *const model = world_->GetModel(goal->id);
  if (model == nullptr) {
    std::ostringstream model_names;
    for (const auto m : world_->GetAllModels()) {
      model_names << ", '" << m->TokenStr() << '\'';
    }
    RCLCPP_ERROR(node_->get_logger(),
                 "tried to move nonexistent model '%s'; existing models are%s",
                 goal->id.c_str(), model_names.str().c_str());

    const auto result = std::make_shared<MoveModel::Result>();
    result->status = MoveModel::Result::STATUS_INVALID_ID;
    goal_handle->abort(result);
    return;
  }

  if (goal->collision_mode != MoveModel::Goal::COLLISION_MODE_IGNORE
      && goal->collision_mode != MoveModel::Goal::COLLISION_MODE_PAUSE
      && goal->collision_mode != MoveModel::Goal::COLLISION_MODE_ABORT) {
    RCLCPP_ERROR(node_->get_logger(),
                 "received invalid collision mode for model '%s'", goal->id.c_str());
    const auto result = std::make_shared<MoveModel::Result>();
    result->status = MoveModel::Result::STATUS_INVALID_VALUE;
    goal_handle->abort(result);
    return;
  }

  if (goal->interpolation_mode != MoveModel::Goal::INTERPOLATION_MODE_LINEAR
      && goal->interpolation_mode != MoveModel::Goal::INTERPOLATION_MODE_CUBIC_SPLINE) {
    RCLCPP_ERROR(node_->get_logger(),
                 "received invalid interpolation mode for model '%s'", goal->id.c_str());
    const auto result = std::make_shared<MoveModel::Result>();
    result->status = MoveModel::Result::STATUS_INVALID_VALUE;
    goal_handle->abort(result);
    return;
  }

  if (!validate_trajectory(*goal)) {
    const auto result = std::make_shared<MoveModel::Result>();
    result->status = MoveModel::Result::STATUS_INVALID_TRAJECTORY;
    goal_handle->abort(result);
    return;
  }

  const rclcpp::Time now = utils::to_ros_time(world_->SimTimeNow());

  MoveModelGoalHandleQueueEntry new_queue_entry{goal_handle, rclcpp::Duration{0, 0}};
  switch (goal->time_resolution_mode) {
    case MoveModel::Goal::TIME_RESOLUTION_MODE_SIM_TIME:
      // Time offset stays 0.
      break;

    case MoveModel::Goal::TIME_RESOLUTION_MODE_RELATIVE_SIM_TIME: {
      new_queue_entry.time_offset = now - rclcpp::Time{0, 0, RCL_ROS_TIME};
      break;
    }

    default: {
      RCLCPP_ERROR(node_->get_logger(),
                   "received invalid time resolution mode for model '%s'", goal->id.c_str());
      const auto result = std::make_shared<MoveModel::Result>();
      result->status = MoveModel::Result::STATUS_INVALID_VALUE;
      goal_handle->abort(result);
      return;
    }
  }

  const rclcpp::Time start_time = new_queue_entry.get_trajectory_time(0);

  MoveModelGoalHandleQueue &q = move_action_goal_handle_queues_by_model_id_[goal->id];
  if (!q.empty() && start_time < q.back().get_end_time()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "trajectory for model '%s' is overlapping previous one in queue",
                 goal->id.c_str());
    const auto result = std::make_shared<MoveModel::Result>();
    result->status = MoveModel::Result::STATUS_OVERLAPPING_TRAJECTORY;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "accepted trajectory for model '%s'", goal->id.c_str());
  q.emplace(std::move(new_queue_entry));
}

ModelServer::RemoveModelStatus ModelServer::do_remove_model(const std::string &id) const {
  Stg::Model *const model = world_->GetModel(id);
  if (model == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "tried to remove nonexistent model '%s'", id.c_str());
    return stage_ros2_itfs::srv::RemoveModel::Response::STATUS_INVALID_ID;
  }

  // This must explicitly be done (because of a bug in the Model destructor) to inform the WorldGui about the
  // removed child:
  utils::get_parent(model)->RemoveChild(model);

  // This may look strange, but the Model destructor will handle the rest of removing the model from the world:
  delete model;

  RCLCPP_INFO(node_->get_logger(), "removed model '%s'", id.c_str());
  return stage_ros2_itfs::srv::RemoveModel::Response::STATUS_SUCCESS;
}

bool ModelServer::validate_trajectory(const MoveModel::Goal &move_model_goal) const {
  const auto &trajectory = move_model_goal.trajectory;
  if (trajectory.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "trajectory for model '%s' is empty",
                 move_model_goal.id.c_str());
    return false;
  }

  if (trajectory.size() > 1) {
    bool trajectory_valid = true;
    for (size_t i = 0; i < (trajectory.size() - 1); ++i) {
      if (rclcpp::Time{trajectory[i + 1].header.stamp} <= trajectory[i].header.stamp) {
        RCLCPP_ERROR(node_->get_logger(),
                    "trajectory for model '%s' does not have strictly increasing timestamps",
                    move_model_goal.id.c_str());
        trajectory_valid = false;
      }
    }

    if (!trajectory_valid) {
      for (const auto &pose : trajectory) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                    "Pose position x: " << pose.pose.position.x << " y: " << pose.pose.position.y <<
                    " orientation z: " << pose.pose.orientation.z << " w: " << pose.pose.orientation.w <<
                    " timestamp seconds: " << pose.header.stamp.sec << " nanosec: " << pose.header.stamp.nanosec);
      }

      return trajectory_valid;
    }
  }

  return true;
}

void ModelServer::enqueue_control_callback() {
  // Enqueue callback in queue 0, which is processed once per simulation step before anything else:
  world_->Enqueue(0, 1, nullptr, [](Stg::Model *, void *user_data) -> int {
    static_cast<ModelServer *>(user_data)->control();
    return 0;
  }, this);
}

void ModelServer::control() {
  // Enqueue callback for next update cycle:
  enqueue_control_callback();

  for (auto entry = move_action_goal_handle_queues_by_model_id_.begin();
       entry != move_action_goal_handle_queues_by_model_id_.end();) {
    Stg::Model *const model = world_->GetModel(entry->first);
    if (model != nullptr) {
      control_model(model, entry->second);
      ++entry;
    } else {
      if (!entry->second.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "model '%s' just disappeared", entry->first.c_str());
        while (!entry->second.empty()) {
          const auto result = std::make_shared<MoveModel::Result>();
          result->status = MoveModel::Result::STATUS_INVALID_ID;
          entry->second.front().goal_handle->abort(result);
          entry->second.pop();
        }
      }

      entry = move_action_goal_handle_queues_by_model_id_.erase(entry);
    }
  }
}

void ModelServer::control_model(Stg::Model *const model,
                                MoveModelGoalHandleQueue &goal_handle_queue) const {
  const rclcpp::Time now = utils::to_ros_time(world_->SimTimeNow());

  while (!goal_handle_queue.empty()) {
    MoveModelGoalHandleQueueEntry &goal_handle_queue_entry = goal_handle_queue.front();
    MoveModelGoalHandleSharedPtr goal_handle = goal_handle_queue_entry.goal_handle;
    const MoveModel::Goal::ConstSharedPtr goal = goal_handle->get_goal();

    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "canceled movement of model '%s'", goal->id.c_str());
      const auto result = std::make_shared<MoveModel::Result>();
      result->status = MoveModel::Result::STATUS_CANCELED;
      goal_handle->canceled(result);
      goal_handle_queue.pop();
      // Continuing with next goal, if present.
    } else {
      if (now < goal_handle_queue_entry.get_trajectory_time(0)) {
        // Nothing to do here yet.
        break;
      }

      if (now < goal_handle_queue_entry.get_end_time()) {
        rcl_duration_value_t iteration = 0;
        rclcpp::Time now_wrt_iteration = now;
        if (1 < goal->iterations) {
          const rclcpp::Time start_time = goal_handle_queue_entry.get_trajectory_time(0);
          const rclcpp::Time end_time
              = goal_handle_queue_entry.get_trajectory_time(goal->trajectory.size() - 1);
          const auto duration_per_iteration = (end_time - start_time).nanoseconds();
          iteration = (now - start_time).nanoseconds() / duration_per_iteration;
          now_wrt_iteration
              -= rclcpp::Duration::from_nanoseconds(duration_per_iteration * iteration);
        }

        size_t trajectory_index = 0;
        for (size_t i = 1; i < (goal->trajectory.size() - 1); ++i) {
          if (now_wrt_iteration < goal_handle_queue_entry.get_trajectory_time(i)) {
            break;
          }
          trajectory_index = i;
        }

        const Stg::Pose pose_a = utils::to_stage_pose(goal->trajectory[trajectory_index].pose);
        const rclcpp::Time stamp_a = goal_handle_queue_entry.get_trajectory_time(trajectory_index);
        const Stg::Pose pose_b = utils::to_stage_pose(goal->trajectory[trajectory_index + 1].pose);
        const rclcpp::Time stamp_b
            = goal_handle_queue_entry.get_trajectory_time(trajectory_index + 1);
        const double f_b = (now_wrt_iteration - stamp_a).seconds() / (stamp_b - stamp_a).seconds();
        const double f_a = 1.0 - f_b;

        const Stg::Pose old_pose = model->GetPose();

        if (goal->interpolation_mode == MoveModel::Goal::INTERPOLATION_MODE_CUBIC_SPLINE
            && (1e-3 <= std::max(std::abs(pose_b.x - pose_a.x), std::abs(pose_b.y - pose_a.y)))) {
          Eigen::Array22d points;
          points.col(0) << pose_a.x, pose_a.y;
          points.col(1) << pose_b.x, pose_b.y;
          Eigen::Array22d derivatives;
          derivatives.col(0) << std::cos(pose_a.a), std::sin(pose_a.a);
          derivatives.col(1) << std::cos(pose_b.a), std::sin(pose_b.a);
          derivatives *= std::hypot(pose_b.x - pose_a.x, pose_b.y - pose_a.y);
          Eigen::Array2i derivative_indices{0, 1};
          const auto spline
              = Eigen::SplineFitting<Eigen::Spline2d>::InterpolateWithDerivatives(
                  points, derivatives, derivative_indices, 3);
          const auto results = spline.derivatives<1>(f_b);

          model->SetPose(Stg::Pose{results.col(0).x(), results.col(0).y(),
                                   pose_a.z * f_a + pose_b.z * f_b,
                                   std::atan2(results.col(1).y(), results.col(1).x())});
        } else {
          const double yaw_diff = Stg::normalize(pose_b.a - pose_a.a);

          model->SetPose(Stg::Pose{pose_a.x * f_a + pose_b.x * f_b,
                                   pose_a.y * f_a + pose_b.y * f_b,
                                   pose_a.z * f_a + pose_b.z * f_b,
                                   pose_a.a + yaw_diff * f_b});
        }

        if (goal->collision_mode != MoveModel::Goal::COLLISION_MODE_IGNORE
            && model->HasCollision()) {
          // Note: if collision_mode is COLLISION_MODE_IGNORE, the model is moved along anyway,
          // possibly through other models.

          model->SetStall(true);
          model->SetPose(old_pose);

          switch (goal->collision_mode) {
            case MoveModel::Goal::COLLISION_MODE_PAUSE: {
              // Increase time offset by simulation interval:
              goal_handle_queue_entry.time_offset = goal_handle_queue_entry.time_offset
                  + utils::to_ros_duration(world_->sim_interval);

              const auto feedback = std::make_shared<MoveModel::Feedback>();
              feedback->iteration = iteration;
              feedback->trajectory_index = trajectory_index;
              goal_handle->publish_feedback(feedback);
              break;
            }

            case MoveModel::Goal::COLLISION_MODE_ABORT: {
              RCLCPP_INFO(node_->get_logger(),
                          "aborted movement of model '%s' because of collision", goal->id.c_str());
              const auto result = std::make_shared<MoveModel::Result>();
              result->status = MoveModel::Result::STATUS_IN_COLLISION;
              goal_handle->abort(result);
              goal_handle_queue.pop();
              break;
            }
          }
        } else {
          model->SetStall(false);

          const auto feedback = std::make_shared<MoveModel::Feedback>();
          feedback->iteration = iteration;
          feedback->trajectory_index = trajectory_index;
          goal_handle->publish_feedback(feedback);
        }

        // Finished with this model and its queue.
        break;
      }

      // Reached the end of this trajectory.
      model->SetPose(utils::to_stage_pose(goal->trajectory.back().pose));

      const auto feedback = std::make_shared<MoveModel::Feedback>();
      feedback->iteration = goal->iterations - 1;
      feedback->trajectory_index = goal->trajectory.size() - 1;
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(node_->get_logger(), "finished movement of model '%s'", goal->id.c_str());
      goal_handle->succeed(std::make_shared<MoveModel::Result>());
      goal_handle_queue.pop();
      // Continuing with next goal, if present.
    }
  }
}

ModelServer::MoveModelGoalHandleQueueEntry::MoveModelGoalHandleQueueEntry(
    MoveModelGoalHandleSharedPtr _goal_handle, const rclcpp::Duration &_time_offset)
    : goal_handle(std::move(_goal_handle)), time_offset(_time_offset) {
}

rclcpp::Time ModelServer::MoveModelGoalHandleQueueEntry::get_trajectory_time(size_t index) const {
  return rclcpp::Time(goal_handle->get_goal()->trajectory.at(index).header.stamp) + time_offset;
}

rclcpp::Time ModelServer::MoveModelGoalHandleQueueEntry::get_end_time() const {
  const auto goal = goal_handle->get_goal();
  if (goal->iterations == MoveModel::Goal::ITERATIONS_INFINITE) {
    return rclcpp::Time{rclcpp::Time::max(), RCL_ROS_TIME};
  }

  const rclcpp::Time end_time = get_trajectory_time(goal->trajectory.size() - 1);

  if (goal->iterations <= 1) {
    return end_time;
  }

  const rclcpp::Time start_time = get_trajectory_time(0);

  const auto total_duration = (end_time - start_time).nanoseconds() * goal->iterations;
  return start_time + rclcpp::Duration::from_nanoseconds(total_duration);
}

}
