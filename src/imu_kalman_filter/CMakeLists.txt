cmake_minimum_required(VERSION 3.8)
project(imu_kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(main_bot_hardware_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(imu_kalman src/imu_kalman.cpp)
target_compile_features(imu_kalman PUBLIC cxx_std_17)
target_include_directories(imu_kalman PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(imu_kalman_node src/imu_kalman_node.cpp)
target_compile_features(imu_kalman_node PUBLIC cxx_std_17)
target_link_libraries(imu_kalman_node imu_kalman)
ament_target_dependencies(imu_kalman_node
  diagnostic_msgs
  main_bot_hardware_msgs
  rclcpp
  sensor_msgs)

install(TARGETS imu_kalman imu_kalman_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_kalman test/test_imu_kalman.cpp)
  target_link_libraries(test_imu_kalman imu_kalman)
endif()

ament_export_include_directories(include)
ament_export_libraries(imu_kalman)
ament_package()
