cmake_minimum_required(VERSION 3.16)
project(megadog_controller)

set(CMAKE_BUILD_TYPE Release)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(megadog_wbc REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

set(MEGADOG_CONTROLLER_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_msgs
  tf2_ros
  geometry_msgs
  std_msgs
  megadog_wbc
)

add_library(${PROJECT_NAME} SHARED
  src/MegadogController.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${MEGADOG_CONTROLLER_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface megadog_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${MEGADOG_CONTROLLER_DEPENDS})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
