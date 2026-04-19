#include "picking_system_core/behavior_tree/action_gripper_control.hpp"

namespace picking_system_core
{

GripperControl::GripperControl(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), ros_node_(node)
{
  gripper_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(
    "/franka/joint_command", 10);
}

BT::PortsList GripperControl::providedPorts()
{
  return {BT::InputPort<std::string>("action", "open", "open 或 close")};
}

BT::NodeStatus GripperControl::tick()
{
  std::string action;
  getInput("action", action);

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = ros_node_->get_clock()->now();

  // 只发夹爪指令：position[7] = 255(闭合) 或 0(张开)
  // mujoco_bridge 里判断 len >= 8 才写 ctrl[7]
  msg.position.resize(8, 0.0);

  if (action == "open") {
    msg.position[7] = 0.0;   // 张开
    RCLCPP_INFO(ros_node_->get_logger(), "[GripperControl] 张开夹爪");
  } else if (action == "close") {
    msg.position[7] = 255.0; // 闭合
    RCLCPP_INFO(ros_node_->get_logger(), "[GripperControl] 闭合夹爪");
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "[GripperControl] 未知动作: %s", action.c_str());
    return BT::NodeStatus::FAILURE;
  }

  gripper_pub_->publish(msg);

  // 等待夹爪动作完成
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace picking_system_core