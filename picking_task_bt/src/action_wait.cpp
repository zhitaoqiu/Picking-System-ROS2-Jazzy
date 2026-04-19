#include "picking_task_bt/action_wait.hpp"

#include <chrono>
#include <thread>

namespace picking_task_bt
{
WaitAction::WaitAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList WaitAction::providedPorts()
{
  return {
    BT::InputPort<int>("milliseconds", 1000, "sleep duration in milliseconds")
  };
}

BT::NodeStatus WaitAction::tick()
{
  int milliseconds = 1000;
  getInput("milliseconds", milliseconds);
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  return BT::NodeStatus::SUCCESS;
}
}  // namespace picking_task_bt