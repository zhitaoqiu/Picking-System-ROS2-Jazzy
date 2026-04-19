#include "picking_task_bt/action_start_servo.hpp"

namespace picking_task_bt
{
StartServoAction::StartServoAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_)
  {
    throw BT::RuntimeError("StartServoAction: missing blackboard entry [node]");
  }
  client_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(
    "/servo_node/switch_command_type");
}

BT::PortsList StartServoAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "command_type", std::string("pose"), "pose | twist | joint_jog")
  };
}

BT::NodeStatus StartServoAction::tick()
{
  std::string command_type = "pose";
  getInput("command_type", command_type);

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(), "StartServoAction: /servo_node/switch_command_type not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();

  if (command_type == "pose")
  {
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
  }
  else if (command_type == "twist")
  {
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
  }
  else
  {
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
  }

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2)) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "StartServoAction: service call failed");
    return BT::NodeStatus::FAILURE;
  }

  return future.get()->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace picking_task_bt