cmake_minimum_required(VERSION 3.8)
project(picking_system_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# 1. Dependencies
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# ==========================================
# 2. Include dirs
# ==========================================
include_directories(include)

# ==========================================
# 3. BehaviorTree 主执行器
# ==========================================
add_executable(bt_main_executor
  src/bt_main_executor.cpp
  src/behavior_tree/action_activate_camera.cpp
  src/behavior_tree/action_detect_target.cpp
  src/behavior_tree/action_execute_grasp.cpp
  src/behavior_tree/action_gripper_control.cpp
)
target_include_directories(bt_main_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_main_executor
  rclcpp
  ament_index_cpp
  behaviortree_cpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

# ==========================================
# 4. Install
# ==========================================
install(TARGETS
  bt_main_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mujoco_bridge.py
  scripts/pinocchio_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch behavior_trees config simulation_assets
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# 5. Export
# ==========================================
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_package()