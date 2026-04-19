#pragma once

#include <behaviortree_cpp/action_node.h>

namespace picking_task_bt
{
class WaitAction : public BT::SyncActionNode
{
public:
  WaitAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
}  // namespace picking_task_bt
