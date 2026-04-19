#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace picking_task_bt
{
class PublishPhaseAction : public BT::SyncActionNode
{
public:
  PublishPhaseAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}  // namespace picking_task_bt
