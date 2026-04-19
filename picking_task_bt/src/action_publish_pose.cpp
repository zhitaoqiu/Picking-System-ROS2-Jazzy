#include "picking_task_bt/action_publish_pose.hpp"

namespace picking_task_bt
{
PublishPoseAction::PublishPoseAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_)
  {
    throw BT::RuntimeError("PublishPoseAction: missing blackboard entry [node]");
  }
  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/servo/pose_cmd", 10);
}

BT::PortsList PublishPoseAction::providedPorts()
{
  return {
    BT::InputPort<double>("x", "target x"),
    BT::InputPort<double>("y", "target y"),
    BT::InputPort<double>("z", "target z"),
    BT::InputPort<double>("qx", 0.0, "orientation x"),
    BT::InputPort<double>("qy", 0.0, "orientation y"),
    BT::InputPort<double>("qz", 0.0, "orientation z"),
    BT::InputPort<double>("qw", 1.0, "orientation w"),
    BT::InputPort<std::string>("frame_id", std::string("panda_link0"), "reference frame")
  };
}

BT::NodeStatus PublishPoseAction::tick()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();

  if (!getInput("frame_id", msg.header.frame_id))
  {
    msg.header.frame_id = "panda_link0";
  }

  if (!getInput("x", msg.pose.position.x) ||
      !getInput("y", msg.pose.position.y) ||
      !getInput("z", msg.pose.position.z))
  {
    throw BT::RuntimeError("PublishPoseAction: missing required input [x/y/z]");
  }

  getInput("qx", msg.pose.orientation.x);
  getInput("qy", msg.pose.orientation.y);
  getInput("qz", msg.pose.orientation.z);
  getInput("qw", msg.pose.orientation.w);

  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace picking_task_bt
