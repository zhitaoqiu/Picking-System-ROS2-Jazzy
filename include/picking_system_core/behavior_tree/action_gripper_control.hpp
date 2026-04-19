#pragma once
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace picking_system_core
{

/**
 * GripperControl
 * 通过 /franka/joint_command 控制夹爪开合
 * action="open"  : 张开夹爪
 * action="close" : 闭合夹爪
 */
class GripperControl : public BT::SyncActionNode
{
public:
  GripperControl(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_pub_;
};

}  // namespace picking_system_core