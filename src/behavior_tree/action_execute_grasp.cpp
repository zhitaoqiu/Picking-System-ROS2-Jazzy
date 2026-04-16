#include "picking_system_core/behavior_tree/action_execute_grasp.hpp"

namespace picking_system_core
{

// 【新增】注册端口，让 XML 可以传参
BT::PortsList ExecuteGrasp::providedPorts()
{
  return { BT::InputPort<std::string>("action") };
}

ExecuteGrasp::ExecuteGrasp(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), ros_node_(node)
{
  // 初始化发布者
  pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);
  vision_enable_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/vision/enable", 10);
  
  // 【核心机制】：暗中监听目标坐标话题，缓存 X 和 Y
  pose_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Pose>(
    "target_pose", 10,
    [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
      last_target_pose_ = *msg;
      has_valid_pose_ = true;
    });
}

BT::NodeStatus ExecuteGrasp::tick()
{
  // 获取 XML 里配置的动作类型
  std::string action;
  if (!getInput("action", action)) {
    action = "plunge"; // 默认防呆设计
  }

  // ================= 阶段一：夺权下扑 =================
  if (action == "plunge") {
    if (!has_valid_pose_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "[大脑] 致命错误：视觉未提供有效坐标，拒绝下扑！");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(ros_node_->get_logger(), "[大脑] 夺权开始：静音视觉节点！");
    std_msgs::msg::Bool disable_msg;
    disable_msg.data = false;
    vision_enable_pub_->publish(disable_msg);

    RCLCPP_INFO(ros_node_->get_logger(), "[大脑] 保持当前 XY，Z 轴突刺至 0.035...");
    geometry_msgs::msg::Pose plunge_pose = last_target_pose_;
    plunge_pose.position.z = 0.035; // 抓取高度 (结合你测得的安全余量微调)
    pose_pub_->publish(plunge_pose);

    return BT::NodeStatus::SUCCESS;
  }
  
  // ================= 阶段二：抬升归还 =================
  else if (action == "lift") {
    RCLCPP_INFO(ros_node_->get_logger(), "[大脑] 抓取完成，Z 轴抬升至 0.200...");
    geometry_msgs::msg::Pose lift_pose = last_target_pose_;
    lift_pose.position.z = 0.200; // 悬空安全高度
    pose_pub_->publish(lift_pose);

    RCLCPP_INFO(ros_node_->get_logger(), "[大脑] 归还控制权：唤醒视觉节点。");
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = true;
    vision_enable_pub_->publish(enable_msg);

    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(ros_node_->get_logger(), "[大脑] 未知动作指令: %s", action.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace picking_system_core