#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/rclcpp.hpp>

namespace picking_task_bt
{
class StartServoAction : public BT::SyncActionNode
{
public:
  StartServoAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr client_;
};
}  // namespace picking_task_bt
