#include "picking_system_core/behavior_tree/action_activate_camera.hpp"

namespace picking_system_core
{

ActivateCamera::ActivateCamera(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), ros_node_(node)
{
  enable_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    "/franka/controller_enable", 10);
}

BT::NodeStatus ActivateCamera::tick()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  enable_pub_->publish(msg);

  RCLCPP_INFO(ros_node_->get_logger(), "[ActivateController] Pinocchio 控制器已启用");

  // 等待控制器就绪
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace picking_system_core