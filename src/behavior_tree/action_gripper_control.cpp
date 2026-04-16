#include "picking_system_core/behavior_tree/action_gripper_control.hpp"

namespace picking_system_core
{
GripperControl::GripperControl(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), ros_node_(node) {}

BT::NodeStatus GripperControl::tick()
{
  std::string action;
  if (!getInput<std::string>("action", action)) {
    return BT::NodeStatus::FAILURE;
  }

  auto client = ros_node_->create_client<std_srvs::srv::SetBool>("/switch_gripper");
  
  RCLCPP_INFO(ros_node_->get_logger(), "[行为树] 正在请求爪子动作: %s", action.c_str());

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(ros_node_->get_logger(), "无法连接到爪子服务！");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = (action == "close"); // close 为 true, open 为 false

  auto result = client->async_send_request(request);
  // 同步等待结果
  if (rclcpp::spin_until_future_complete(ros_node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}