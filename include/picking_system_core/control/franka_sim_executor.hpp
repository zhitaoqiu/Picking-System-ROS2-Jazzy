/*把 C++ 命令发到 ROS 2 里，交给 Python bridge*/
#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "picking_system_core/simulation/sim_types.hpp"

namespace picking_system_core::control
{
class FrankaSimExecutor
{
public:
    explicit FrankaSimExecutor(const rclcpp::Node::SharedPtr& node);
    bool sendPoseCommand(const simulation::PoseCommand& command);
    bool sendJointCommand(const simulation::JointCommand& command);
    bool sendGripperCommand(const simulation::GripperCommand& command);
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sim_command_pub_;
    std::string createPoseCommandMessage(const simulation::PoseCommand& command) const;
    std::string createJointCommandMessage(const simulation::JointCommand& command) const;
    std::string createGripperCommandMessage(const simulation::GripperCommand& command) const;

};
} // namespace picking_system_core::control