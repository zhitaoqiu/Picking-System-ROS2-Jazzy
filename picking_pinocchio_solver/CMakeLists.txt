cmake_minimum_required(VERSION 3.8)
project(picking_pinocchio_solver)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/pinocchio_solver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

