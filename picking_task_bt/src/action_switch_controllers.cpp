#include "picking_task_bt/action_switch_controllers.hpp"

#include <sstream>

namespace
{
std::vector<std::string> split(const std::string & input)
{
  std::vector<std::string> out;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, ';'))
  {
    if (!item.empty())
    {
      out.push_back(item);
    }
  }
  return out;
}
}  // namespace

namespace picking_task_bt
{
SwitchControllersAction::SwitchControllersAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_)
  {
    throw BT::RuntimeError("SwitchControllersAction: missing blackboard entry [node]");
  }
  client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");
}

BT::PortsList SwitchControllersAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "activate", std::string(""), "semicolon-separated controller names to activate"),
    BT::InputPort<std::string>(
      "deactivate", std::string(""), "semicolon-separated controller names to deactivate")
  };
}

BT::NodeStatus SwitchControllersAction::tick()
{
  std::string activate;
  std::string deactivate;
  getInput("activate", activate);
  getInput("deactivate", deactivate);

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(), "SwitchControllersAction: /controller_manager/switch_controller not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = split(activate);
  request->deactivate_controllers = split(deactivate);
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout.sec = 5;
  request->timeout.nanosec = 0;

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(6)) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "SwitchControllersAction: service call failed");
    return BT::NodeStatus::FAILURE;
  }

  return future.get()->ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace picking_task_bt