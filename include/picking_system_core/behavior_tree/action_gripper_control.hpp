#ifndef PICKING_SYSTEM_CORE__ACTION_GRIPPER_CONTROL_HPP_
#define PICKING_SYSTEM_CORE__ACTION_GRIPPER_CONTROL_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace picking_system_core
{
class GripperControl : public BT::SyncActionNode
{
public:
  GripperControl(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);
  static BT::PortsList providedPorts() {
    // 增加一个输入端口：action（输入 "open" 或 "close"）
    return { BT::InputPort<std::string>("action") };
  }
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
};
}
#endif