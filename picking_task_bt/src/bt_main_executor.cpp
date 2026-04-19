#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <rclcpp/rclcpp.hpp>

#include "picking_task_bt/action_pause_servo.hpp"
#include "picking_task_bt/action_publish_phase.hpp"
#include "picking_task_bt/action_publish_pose.hpp"
#include "picking_task_bt/action_start_servo.hpp"
#include "picking_task_bt/action_switch_controllers.hpp"
#include "picking_task_bt/action_wait.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_main_executor");

  node->declare_parameter<std::string>(
    "tree_file",
    ament_index_cpp::get_package_share_directory("picking_task_bt") + "/behavior_trees/peg_in_hole_servo.xml");
  const auto tree_file = node->get_parameter("tree_file").as_string();

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<picking_task_bt::StartServoAction>("StartServo");
  factory.registerNodeType<picking_task_bt::PublishPoseAction>("PublishPose");
  factory.registerNodeType<picking_task_bt::PauseServoAction>("PauseServo");
  factory.registerNodeType<picking_task_bt::SwitchControllersAction>("SwitchControllers");
  factory.registerNodeType<picking_task_bt::WaitAction>("WaitMs");
  factory.registerNodeType<picking_task_bt::PublishPhaseAction>("PublishPhase");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  auto tree = factory.createTreeFromFile(tree_file, blackboard);

  rclcpp::Rate rate(10.0);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = tree.tickOnce();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Behavior tree finished with status: %s",
              BT::toStr(status, true).c_str());

  rclcpp::shutdown();
  return 0;
}
