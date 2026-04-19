#include "picking_system_core/control/franka_sim_executor.hpp"
#include <sstream>


namespace picking_system_core::control
{
FrankaSimExecutor::FrankaSimExecutor(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    {
    sim_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/mujuco/command", 10);
    }
bool FrankaSimExecutor::sendPoseCommand(const simulation::PoseCommand& command)
{
    std_msgs::msg::String msg;
    msg.data = createPoseCommandMessage(command);
    sim_command_pub_->publish(msg);
    return true;
}
bool FrankaSimExecutor::sendJointCommand(const simulation::JointCommand& command)
{
    std_msgs::msg::String msg;
    msg.data = createJointCommandMessage(command);
    sim_command_pub_->publish(msg);
    return true;
}
bool FrankaSimExecutor::sendGripperCommand(const simulation::GripperCommand& command)
{
    std_msgs::msg::String msg;
    msg.data = createGripperCommandMessage(command);
    sim_command_pub_->publish(msg);
    return true;
}
std::string FrankaSimExecutor::createPoseCommandMessage(const simulation::PoseCommand& command) const
{
    std::ostringstream oss;
      oss << "POSE "
      << command.x << " "
      << command.y << " "
      << command.z << " "
      << command.qx << " "
      << command.qy << " "
      << command.qz << " "
      << command.qw;
  return oss.str();
}
std::string FrankaSimExecutor::createJointCommandMessage(const simulation::JointCommand& command) const
{
    std::ostringstream oss;
    oss << "JOINT ";
    for (const auto& position : command.positions)
    {
        oss << position << " ";
    }
    return oss.str();
}
std::string FrankaSimExecutor::createGripperCommandMessage(const simulation::GripperCommand& command) const
{
    std::ostringstream oss;
    oss << "GRIPPER "    << command.width << " " << command.force;
    return oss.str();
}
} // namespace picking_system_core::control