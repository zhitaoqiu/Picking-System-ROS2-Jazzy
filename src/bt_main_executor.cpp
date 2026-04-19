#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "picking_system_core/behavior_tree/action_activate_camera.hpp"
#include "picking_system_core/behavior_tree/action_detect_target.hpp"
#include "picking_system_core/behavior_tree/action_execute_grasp.hpp"
#include "picking_system_core/behavior_tree/action_gripper_control.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_main_executor");

  BT::BehaviorTreeFactory factory;

  // 注册所有 Action 节点（统一注入 node 指针）
  auto builder = [&node](auto * /*unused*/) {};
  (void)builder;

  factory.registerBuilder<picking_system_core::ActivateCamera>(
    "ActivateCamera",
    [&node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<picking_system_core::ActivateCamera>(name, config, node);
    });

  factory.registerBuilder<picking_system_core::DetectTarget>(
    "DetectTarget",
    [&node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<picking_system_core::DetectTarget>(name, config, node);
    });

  factory.registerBuilder<picking_system_core::ExecuteGrasp>(
    "ExecuteGrasp",
    [&node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<picking_system_core::ExecuteGrasp>(name, config, node);
    });

  factory.registerBuilder<picking_system_core::GripperControl>(
    "GripperControl",
    [&node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<picking_system_core::GripperControl>(name, config, node);
    });

  // 加载行为树 XML
  std::string pkg_share = ament_index_cpp::get_package_share_directory("picking_system_core");
  std::string xml_path = pkg_share + "/behavior_trees/main_picking_tree.xml";

  RCLCPP_INFO(node->get_logger(), "加载行为树: %s", xml_path.c_str());

  try {
    auto tree = factory.createTreeFromFile(xml_path);

    rclcpp::Rate rate(10);  // 10Hz tick
    while (rclcpp::ok()) {
      BT::NodeStatus status = tree.tickOnce();
      rclcpp::spin_some(node);

      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "✅ 任务完成！");
        break;
      } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(node->get_logger(), "❌ 任务失败！");
        break;
      }

      rate.sleep();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "行为树异常: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}