#include "picking_system_core/control/robot_executor.hpp"
namespace picking_system_core::control
{
RobotExecutor::RobotExecutor(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    {
    sim_executor_ = std::make_shared<FrankaSimExecutor>(node_);
    }
bool RobotExecutor::moveToPose(const simulation::PoseCommand& command)
{
    RCLCPP_INFO(node_->get_logger(),"Moving to pose: x=%.2f, y=%.2f, z=%.2f", command.x, command.y, command.z);
    return sim_executor_->sendPoseCommand(command);
}
bool RobotExecutor::moveToJointPosition(const simulation::JointCommand& command)
{    
    RCLCPP_INFO(node_->get_logger(),"Moving to joint positions: [%s]", 
        std::to_string(command.positions[0]).c_str());
    return sim_executor_->sendJointCommand(command);
}
bool RobotExecutor::controlGripper(const simulation::GripperCommand& command)
{    
    RCLCPP_INFO(node_->get_logger(),"Controlling gripper: width=%.2f", command.width);
    return sim_executor_->sendGripperCommand(command);
}
} // namespace picking_system_core::control