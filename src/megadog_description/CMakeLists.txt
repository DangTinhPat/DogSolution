cmake_minimum_required(VERSION 3.8)
project(megadog_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/kill_gz.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
