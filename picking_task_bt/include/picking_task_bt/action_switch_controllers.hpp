#pragma once

#include <behaviortree_cpp/action_node.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace picking_task_bt
{
class SwitchControllersAction : public BT::SyncActionNode
{
public:
  SwitchControllersAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_;
};
}  // namespace picking_task_bt
