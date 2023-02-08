#ifndef STAGE_ROS2_MODEL_SERVER_HPP_
#define STAGE_ROS2_MODEL_SERVER_HPP_

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
  void execute_move_model_goal(const MoveModelGoalHandleSharedPtr& goal_handle);

  RemoveModelStatus do_remove_model(const std::string &id) const;
  bool validate_trajectory(const MoveModel::Goal::_trajectory_type& trajectory) const;
  void enqueue_control_callback();
  void control();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Stg::World> world_;

  rclcpp::Service<CreateModel>::SharedPtr create_model_service_;
  rclcpp::Service<RemoveModel>::SharedPtr remove_model_service_;
  rclcpp::Service<RemoveModels>::SharedPtr remove_models_service_;
  rclcpp_action::Server<MoveModel>::SharedPtr move_model_action_server_;
  MoveModelGoalHandleQueueMap move_action_goal_handle_queues_by_model_id_;
};

#endif //STAGE_ROS2_MODEL_SERVER_HPP_
