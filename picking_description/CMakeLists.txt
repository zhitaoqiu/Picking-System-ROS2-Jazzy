cmake_minimum_required(VERSION 3.8)
project(picking_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf mjcf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

