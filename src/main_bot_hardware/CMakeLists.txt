cmake_minimum_required(VERSION 3.8)
project(main_bot_hardware)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HARDWARE_INCLUDE_DEPENDS
        pluginlib
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        main_bot_hardware_msgs
)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${HARDWARE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED
        src/real_system.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${HARDWARE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface main_bot_hardware.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

ament_export_dependencies(${HARDWARE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
