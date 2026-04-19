#pragma once
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace picking_system_core
{

/**
 * DetectTarget
 * 发布 peg-in-hole 的目标位姿到 /franka/target_pose
 * 在真实场景中这里可以接视觉检测结果，现在直接用固定坐标
 */
class DetectTarget : public BT::SyncActionNode
{
public:
  DetectTarget(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
};

}  // namespace picking_system_core