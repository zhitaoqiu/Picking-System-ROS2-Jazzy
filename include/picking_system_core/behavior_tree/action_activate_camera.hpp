#ifndef PICKING_SYSTEM_CORE__ACTION_ACTIVATE_CAMERA_HPP_
#define PICKING_SYSTEM_CORE__ACTION_ACTIVATE_CAMERA_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace picking_system_core
{
class ActivateCamera : public BT::SyncActionNode
{
public:
  ActivateCamera(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};
}  // namespace picking_system_core

#endif  // PICKING_SYSTEM_CORE__ACTION_ACTIVATE_CAMERA_HPP_