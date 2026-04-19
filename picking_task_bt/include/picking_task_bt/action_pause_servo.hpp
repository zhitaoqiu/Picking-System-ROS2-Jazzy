#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace picking_task_bt
{
class PauseServoAction : public BT::SyncActionNode
{
public:
  PauseServoAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};
}  // namespace picking_task_bt
