#pragma once
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace picking_system_core
{

/**
 * ActivateController
 * 发送 controller_enable=True 给 pinocchio_controller
 * 保留文件名 action_activate_camera 以免改动太多
 */
class ActivateCamera : public BT::SyncActionNode
{
public:
  ActivateCamera(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
};

}  // namespace picking_system_core