cmake_minimum_required(VERSION 3.16)
project(megadog_legged_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

set(MEGADOG_LEGGED_INTERFACE_DEPS
  ocs2_core
  ocs2_oc
  ocs2_mpc
  ocs2_sqp
  ocs2_ddp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_legged_robot
)

# --- Generate a plain URDF from megadog_description's urdf/robot.xacro -----
# OCS2/Pinocchio need a literal URDF, not xacro. Source-tree relative path
# (not $(find)) for the same reason as babydog_legged_interface: avoids a
# colcon dependency cycle once a controller package depends on both this
# package and megadog_description.
find_program(XACRO_EXECUTABLE xacro REQUIRED)
get_filename_component(MEGADOG_DESCRIPTION_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../megadog_description" ABSOLUTE)
set(MEGADOG_A1_XACRO_INPUT "${MEGADOG_DESCRIPTION_SOURCE_DIR}/urdf/robot.xacro")
set(MEGADOG_A1_GENERATED_URDF_PATH "${CMAKE_CURRENT_BINARY_DIR}/a1.urdf")
file(GLOB_RECURSE MEGADOG_A1_XACRO_DEPS CONFIGURE_DEPENDS
  "${MEGADOG_DESCRIPTION_SOURCE_DIR}/urdf/*.xacro"
)

add_custom_command(
  OUTPUT "${MEGADOG_A1_GENERATED_URDF_PATH}"
  COMMAND "${XACRO_EXECUTABLE}" "${MEGADOG_A1_XACRO_INPUT}"
          "robot_type:=a1"
          "megadog_description_share:=${MEGADOG_DESCRIPTION_SOURCE_DIR}"
          -o "${MEGADOG_A1_GENERATED_URDF_PATH}"
  DEPENDS "${MEGADOG_A1_XACRO_DEPS}"
  COMMENT "megadog_legged_interface: generating plain URDF from megadog_description's urdf/robot.xacro"
  VERBATIM
)
add_custom_target(megadog_a1_generate_urdf ALL DEPENDS "${MEGADOG_A1_GENERATED_URDF_PATH}")

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# --- Library -----------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/MegadogA1Interface.cpp
)
add_dependencies(${PROJECT_NAME} megadog_a1_generate_urdf)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${MEGADOG_LEGGED_INTERFACE_DEPS} pinocchio)
target_link_libraries(${PROJECT_NAME} PUBLIC Boost::boost)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${MEGADOG_LEGGED_INTERFACE_DEPS} pinocchio)
ament_export_targets(export_${PROJECT_NAME})

# --- Tests ---------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_megadog_legged_interface test/test_megadog_legged_interface.cpp)
  target_link_libraries(test_megadog_legged_interface ${PROJECT_NAME})
endif()

ament_package()
