#pragma once
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp" // 【新增】开关消息类型

namespace picking_system_core
{
class ExecuteGrasp : public BT::SyncActionNode
{
public:
  ExecuteGrasp(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);
  
  // 【新增】允许从 XML 接收 action 参数
  static BT::PortsList providedPorts();
  
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  
  // 两个发布者：一个发坐标，一个控视觉开关
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_enable_pub_;
  
  // 一个订阅者：用来偷听视觉节点算出的最新坐标
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  
  // 缓存变量
  geometry_msgs::msg::Pose last_target_pose_;
  bool has_valid_pose_ = false;
};
}  // namespace picking_system_core