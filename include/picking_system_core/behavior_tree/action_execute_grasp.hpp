#pragma once
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

namespace picking_system_core
{

/**
 * ExecuteGrasp
 * 控制机械臂执行插孔动作
 * action="approach" : 移动到孔上方待机位
 * action="insert"   : 垂直下插
 * action="retract"  : 抬升退出
 */
class ExecuteGrasp : public BT::SyncActionNode
{
public:
  ExecuteGrasp(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;

  // 等待机械臂到位的超时时间
  static constexpr double SETTLE_TIME_SEC = 2.0;
};

}  // namespace picking_system_core