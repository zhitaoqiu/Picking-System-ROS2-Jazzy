cmake_minimum_required(VERSION 3.8)
project(picking_task_bt)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(bt_main_executor
  src/bt_main_executor.cpp
  src/action_start_servo.cpp
  src/action_publish_pose.cpp
  src/action_pause_servo.cpp
  src/action_switch_controllers.cpp
  src/action_wait.cpp
  src/action_publish_phase.cpp
)

ament_target_dependencies(bt_main_executor
  ament_index_cpp
  behaviortree_cpp
  controller_manager_msgs
  geometry_msgs
  moveit_msgs
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS bt_main_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
