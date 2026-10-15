cmake_minimum_required(VERSION 3.8)
project(tank01_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
