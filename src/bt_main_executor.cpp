#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "picking_system_core/behavior_tree/action_activate_camera.hpp"
#include "picking_system_core/behavior_tree/action_execute_grasp.hpp"
#include "picking_system_core/behavior_tree/action_gripper_control.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_main_executor");

  BT::BehaviorTreeFactory factory;

  // --- 节点注册区 ---

  // 注册相机激活节点 (由于它内部自建了临时 Node 处理同步服务，所以可以用简单注册)
  factory.registerNodeType<picking_system_core::ActivateCamera>("ActivateCamera");

  // 注册抓取执行节点 (注入 node 指针用于发 Topic)
  factory.registerBuilder<picking_system_core::ExecuteGrasp>(
    "ExecuteGrasp",
    [&node](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<picking_system_core::ExecuteGrasp>(name, config, node);
    });

  // 👇 【新增】2. 注册爪子控制节点 (注入 node 指针用于呼叫 Service)
  factory.registerBuilder<picking_system_core::GripperControl>(
    "GripperControl",
    [&node](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<picking_system_core::GripperControl>(name, config, node);
    });

  // ----------------

  // 2. 找到 XML 文件的路径
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("picking_system_core");
  std::string xml_path = pkg_share_dir + "/behavior_trees/main_picking_tree.xml";

  RCLCPP_INFO(node->get_logger(), "正在加载行为树: %s", xml_path.c_str());

  // 3. 创建并执行树
  try {
    auto tree = factory.createTreeFromFile(xml_path);
    
    rclcpp::Rate rate(10); // 10Hz tick
    while (rclcpp::ok()) {
      BT::NodeStatus status = tree.tickOnce();
      
      if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
        RCLCPP_INFO(node->get_logger(), "整体任务结束 (状态: %s)", 
                    status == BT::NodeStatus::SUCCESS ? "成功" : "失败");
        break; 
      }
      
      rclcpp::spin_some(node);
      rate.sleep();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "行为树加载或执行失败: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}