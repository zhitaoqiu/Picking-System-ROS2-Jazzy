#include "picking_system_core/behavior_tree/action_detect_target.hpp"

namespace picking_system_core
{

DetectTarget::DetectTarget(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), ros_node_(node)
{
  target_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/franka/target_pose", 10);
}

BT::PortsList DetectTarget::providedPorts()
{
  // 预留接口：将来可从视觉检测结果 BB 读取坐标
  return {
    BT::InputPort<double>("target_x", 0.50, "目标 X"),
    BT::InputPort<double>("target_y", 0.00, "目标 Y"),
    BT::InputPort<double>("target_z", 0.35, "目标 Z（孔上方待机高度）"),
  };
}

BT::NodeStatus DetectTarget::tick()
{
  double tx, ty, tz;
  getInput("target_x", tx);
  getInput("target_y", ty);
  getInput("target_z", tz);

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = ros_node_->get_clock()->now();
  msg.header.frame_id = "panda_link0";

  msg.pose.position.x = tx;
  msg.pose.position.y = ty;
  msg.pose.position.z = tz;

  // 末端朝下：绕 X 轴旋转 180 度的四元数
  msg.pose.orientation.x = 1.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 0.0;

  target_pub_->publish(msg);

  RCLCPP_INFO(ros_node_->get_logger(),
    "[DetectTarget] 发布目标位姿: [%.3f, %.3f, %.3f]", tx, ty, tz);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace picking_system_core