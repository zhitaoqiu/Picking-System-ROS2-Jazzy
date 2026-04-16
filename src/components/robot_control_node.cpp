#include "picking_system_core/components/robot_control_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace picking_system_core
{

RobotControlNode::RobotControlNode(const rclcpp::NodeOptions & options)
: Node("robot_control_node", options)
{
  RCLCPP_INFO(this->get_logger(), "机械臂组件已加载！等待 2 秒后连接 MoveIt 2...");

  // 1. 初始化订阅者
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "target_pose", 10,
    std::bind(&RobotControlNode::pose_callback, this, std::placeholders::_1));

  // 2. 延迟初始化 MoveIt 2 (避免在 Component 容器启动瞬间抢占资源)
  init_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&RobotControlNode::init_moveit, this)
  );
}

void RobotControlNode::init_moveit()
{
  // 如果已经初始化成功，关闭重试定时器
  if (move_group_) {
    init_timer_->cancel();
    return;
  }

  try {
    // 初始化 MoveGroupInterface (假设你的配置叫 "panda_arm")
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      this->shared_from_this(), "panda_arm");

    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);

    RCLCPP_INFO(this->get_logger(), "MoveIt 2 规划器连接成功！随时待命。");
    init_timer_->cancel();
  } catch (const std::exception & e) {
    // 当 robot_description 未就绪时，不让组件崩溃，保留节点并继续重试
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "MoveIt 2 初始化失败，等待 robot_description 后重试: %s", e.what());
  }
}

void RobotControlNode::pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  if (!move_group_) {
    RCLCPP_WARN(this->get_logger(), "MoveIt 还没准备好，丢弃本次指令！");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "开始规划目标点...");
  move_group_->setPoseTarget(*msg);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "规划成功！执行轨迹...");
    move_group_->execute(my_plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "规划失败！");
  }
}

}  // namespace picking_system_core

RCLCPP_COMPONENTS_REGISTER_NODE(picking_system_core::RobotControlNode)