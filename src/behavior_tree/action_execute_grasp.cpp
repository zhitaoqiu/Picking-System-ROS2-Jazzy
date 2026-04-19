#include "picking_system_core/behavior_tree/action_execute_grasp.hpp"

namespace picking_system_core
{

ExecuteGrasp::ExecuteGrasp(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), ros_node_(node)
{
  pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/franka/target_pose", 10);
  enable_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    "/franka/controller_enable", 10);
}

BT::PortsList ExecuteGrasp::providedPorts()
{
  return {BT::InputPort<std::string>("action", "approach", "approach/insert/retract")};
}

BT::NodeStatus ExecuteGrasp::tick()
{
  std::string action;
  getInput("action", action);

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = ros_node_->get_clock()->now();
  msg.header.frame_id = "panda_link0";

  // 末端朝下的四元数（绕X轴转180度）
  msg.pose.orientation.x = 1.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 0.0;

  if (action == "approach") {
    // 移动到孔正上方 15cm 处待机
    msg.pose.position.x = 0.50;
    msg.pose.position.y = 0.00;
    msg.pose.position.z = 0.35;
    pose_pub_->publish(msg);
    RCLCPP_INFO(ros_node_->get_logger(), "[ExecuteGrasp] approach: 移动到孔上方待机位");

  } else if (action == "insert") {
    // 垂直下插到孔内
    msg.pose.position.x = 0.50;
    msg.pose.position.y = 0.00;
    msg.pose.position.z = 0.23;  // 与项目1的 nominal_target_pos 一致
    pose_pub_->publish(msg);
    RCLCPP_INFO(ros_node_->get_logger(), "[ExecuteGrasp] insert: 执行插入动作");

  } else if (action == "retract") {
    // 抬升退出
    msg.pose.position.x = 0.50;
    msg.pose.position.y = 0.00;
    msg.pose.position.z = 0.50;
    pose_pub_->publish(msg);
    RCLCPP_INFO(ros_node_->get_logger(), "[ExecuteGrasp] retract: 抬升退出");

  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "[ExecuteGrasp] 未知动作: %s", action.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 等待机械臂运动稳定
  rclcpp::sleep_for(std::chrono::milliseconds(
    static_cast<int>(SETTLE_TIME_SEC * 1000)));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace picking_system_core