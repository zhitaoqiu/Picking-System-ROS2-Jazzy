#include "picking_task_bt/action_pause_servo.hpp"

namespace picking_task_bt
{
PauseServoAction::PauseServoAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_)
  {
    throw BT::RuntimeError("PauseServoAction: missing blackboard entry [node]");
  }
  client_ = node_->create_client<std_srvs::srv::SetBool>("/servo_node/pause_servo");
}

BT::PortsList PauseServoAction::providedPorts()
{
  return {
    BT::InputPort<bool>("pause", true, "true=pause, false=resume")
  };
}

BT::NodeStatus PauseServoAction::tick()
{
  bool pause = true;
  getInput("pause", pause);

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(), "PauseServoAction: /servo_node/pause_servo not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = pause;

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2)) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "PauseServoAction: service call failed");
    return BT::NodeStatus::FAILURE;
  }

  return future.get()->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace picking_task_bt