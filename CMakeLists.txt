cmake_minimum_required(VERSION 3.8)
project(picking_system_core)

# 强制使用 C++17 或更高版本 (BT.CPP 和 现代 ROS 2 必备)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 开启严苛的编译器警告 (展现代码质量)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# 1. 寻找系统依赖
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED) # 引入行为树引擎
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# ==========================================
# 2. 编译核心组件 (Components) 为共享库 (.so)
# ==========================================
# 相机 Lifecycle 节点组件
add_library(camera_lifecycle_component SHARED
  src/components/camera_lifecycle_node.cpp
)
target_include_directories(camera_lifecycle_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(camera_lifecycle_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)
# 关键：注册节点为 ROS 2 插件，使其能被 ComponentContainer 动态加载
rclcpp_components_register_node(camera_lifecycle_component
  PLUGIN "picking_system_core::CameraLifecycleNode"
  EXECUTABLE camera_lifecycle_node_exec # 顺便生成一个单机可执行文件方便调试
  
)

# 机械臂控制节点组件 (结构同上)
add_library(robot_control_component SHARED
  src/components/robot_control_node.cpp
)
target_include_directories(robot_control_component PUBLIC include)
ament_target_dependencies(robot_control_component rclcpp rclcpp_components geometry_msgs moveit_ros_planning_interface)
rclcpp_components_register_node(robot_control_component
  PLUGIN "picking_system_core::RobotControlNode"
  EXECUTABLE robot_control_node_exec
)

# ==========================================
# 3. 编译行为树引擎主程序 (Executable)
# ==========================================
add_executable(vision_processor src/vision_processor.cpp)
ament_target_dependencies(vision_processor rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(vision_processor ${OpenCV_LIBRARIES})

install(TARGETS vision_processor
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(bt_main_executor
  src/bt_main_executor.cpp
  src/behavior_tree/action_detect_target.cpp
  src/behavior_tree/action_execute_grasp.cpp
  src/behavior_tree/action_activate_camera.cpp
  src/behavior_tree/action_gripper_control.cpp
)
target_include_directories(bt_main_executor PUBLIC include)
ament_target_dependencies(bt_main_executor
  rclcpp
  behaviortree_cpp
  geometry_msgs
  lifecycle_msgs
  std_srvs
)

# ==========================================
# 4. 安装指令 (Install)
# ==========================================
# 安装目录与库文件
install(TARGETS
  camera_lifecycle_component
  robot_control_component
  bt_main_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装行为树 XML 配置、Launch 和 参数文件
install(DIRECTORY launch behavior_trees config
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# 5. 导出依赖与打包
# ==========================================
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  behaviortree_cpp
)

ament_package()
