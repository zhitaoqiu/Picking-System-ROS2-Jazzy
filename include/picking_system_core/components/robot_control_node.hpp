#ifndef PICKING_SYSTEM_CORE__ROBOT_CONTROL_NODE_HPP_
#define PICKING_SYSTEM_CORE__ROBOT_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
namespace picking_system_core
{

class RobotControlNode : public rclcpp::Node
{
public:
  explicit RobotControlNode(const rclcpp::NodeOptions & options);
  ~RobotControlNode() override = default;

private:
  // 模拟接收抓取目标位姿的订阅者
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr init_timer_;
  void init_moveit();
};

}  // namespace picking_system_core

#endif  // PICKING_SYSTEM_CORE__ROBOT_CONTROL_NODE_HPP_