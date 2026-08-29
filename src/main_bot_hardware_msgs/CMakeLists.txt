cmake_minimum_required(VERSION 3.8)
project(main_bot_hardware_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuRaw.msg"
  "msg/JointCmd.msg"
  "msg/JointDiag.msg"
  "msg/JointFb.msg"
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
