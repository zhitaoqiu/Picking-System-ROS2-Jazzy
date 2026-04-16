#include "picking_system_core/behavior_tree/action_activate_camera.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace picking_system_core
{

ActivateCamera::ActivateCamera(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::NodeStatus ActivateCamera::tick()
{
  // 创建一个临时的独立节点用于发送服务请求 (防止与主线程死锁)
  auto client_node = rclcpp::Node::make_shared("temp_lifecycle_client");
  auto client = client_node->create_client<lifecycle_msgs::srv::ChangeState>("/camera_node/change_state");

  RCLCPP_INFO(client_node->get_logger(), "[行为树] 正在检查并唤醒相机节点...");

  // 等待服务上线
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(client_node->get_logger(), "[行为树] 找不到相机的 Lifecycle 服务！");
    return BT::NodeStatus::FAILURE;
  }

  auto change_state = [&](uint8_t transition_id, const char * transition_name) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_node, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(client_node->get_logger(), "[行为树] %s 请求超时/失败", transition_name);
      return false;
    }
    if (!result.get()->success) {
      RCLCPP_WARN(client_node->get_logger(), "[行为树] %s 被拒绝（可能已处于目标状态）", transition_name);
      return false;
    }
    return true;
  };

  // 先配置，再激活
  const bool configured = change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "Configure");
  const bool activated = change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "Activate");

  if (configured || activated) {
    RCLCPP_INFO(client_node->get_logger(), "[行为树] 相机节点已可用（已配置/激活）。");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(client_node->get_logger(), "[行为树] 相机状态切换失败！");
  return BT::NodeStatus::FAILURE;
}

}  // namespace picking_system_core