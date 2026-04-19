#include "picking_task_bt/action_publish_phase.hpp"

namespace picking_task_bt
{
PublishPhaseAction::PublishPhaseAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_)
  {
    throw BT::RuntimeError("PublishPhaseAction: missing blackboard entry [node]");
  }
  publisher_ = node_->create_publisher<std_msgs::msg::String>("/task/phase", 10);
}

BT::PortsList PublishPhaseAction::providedPorts()
{
  return {BT::InputPort<std::string>("value", "phase string")};
}

BT::NodeStatus PublishPhaseAction::tick()
{
  std::string value;
  if (!getInput("value", value))
  {
    throw BT::RuntimeError("PublishPhaseAction: missing required input [value]");
  }

  std_msgs::msg::String msg;
  msg.data = value;
  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace picking_task_bt