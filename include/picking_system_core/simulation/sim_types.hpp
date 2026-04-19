/*：定义仿真相关公共数据类型*/
#pragma once
#include <string>
#include <vector>

namespace picking_system_core::simulation
{
struct PoseCommand
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{0.0};
};
struct GripperCommand
{
    double width{0.0};
    double force{0.0};
};
struct JointCommand
{
    std::vector<double> positions;
};
struct ExecutionResult
{
    bool success{false};// 是否成功执行默认为false
    std::string message;// 可选的错误信息
};
} // namespace picking_system_core::simulation