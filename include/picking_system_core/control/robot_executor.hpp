/*定义“统一机器人执行器”接口。

它是行为树 / 主流程调用执行动作的入口，先不要让行为树直接调用 MuJoCo 或 Python。

它应该负责抽象这些动作，比如：

移动到抓取前位姿
执行抓取
张开/闭合夹爪
回到安全位*/
#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "picking_system_core/simulation/sim_types.hpp"
#include "picking_system_core/control/franka_sim_executor.hpp"
namespace picking_system_core::control
{
class RobotExecutor
{
public:
    explicit RobotExecutor(const rclcpp::Node::SharedPtr& node);
    bool moveToPose(const simulation::PoseCommand& command);
    bool moveToJointPosition(const simulation::JointCommand& command);
    bool controlGripper(const simulation::GripperCommand& command);
private:
    std::shared_ptr<FrankaSimExecutor> sim_executor_;//智能指针，负责管理FrankaSimExecutor的生命周期
    rclcpp::Node::SharedPtr node_;
};
} // namespace picking_system_core::control