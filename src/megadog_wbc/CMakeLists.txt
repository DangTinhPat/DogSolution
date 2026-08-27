cmake_minimum_required(VERSION 3.16)
project(megadog_wbc)

# Matches babyDog's controller/CMakeLists.txt - this is control-loop-critical
# QP code that wants optimization. NOTE: Release also defines NDEBUG (disables
# Eigen's runtime assertions), which had been silently masking a real type
# bug in the ported WbcBase.cpp (formulateFloatingBaseEomTask was passing a
# matrix_t() where the Task constructor's vector_t f is expected - now fixed
# at the source instead of just relying on this to hide it).
set(CMAKE_BUILD_TYPE Release)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(pinocchio REQUIRED)
find_package(megadog_legged_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# ocs2_legged_robot/ocs2_robotic_tools/ocs2_sqp/ocs2_ipm/hpipm_catkin's
# exported CMake targets reference Boost::system/filesystem/log(_setup) as an
# IMPORTED link interface, which only exists once something in this
# CMakeLists actually calls find_package(Boost ...) itself - a bare
# find_package(ocs2_core REQUIRED) etc. above does not create it (same fix
# babyDog's controller/CMakeLists.txt needed).
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

set(MEGADOG_WBC_DEPENDS
  ocs2_core
  ocs2_oc
  ocs2_mpc
  ocs2_sqp
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_ros_interfaces
  megadog_legged_interface
  rclcpp
  tf2_ros
  geometry_msgs
  visualization_msgs
)

# qpOASES 3.2.1 (LGPL-2.1, vendored - see ../third_party/qpoases/LICENSE),
# copied verbatim from babyDog's src/controller/third_party/qpoases.
file(GLOB QPOASES_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/qpoases/src/*.cpp")
add_library(qpoases STATIC ${QPOASES_SOURCES})
set_target_properties(qpoases PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(qpoases PRIVATE __NO_COPYRIGHT__ LINUX)
target_include_directories(qpoases
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third_party/qpoases/include>"
)

add_library(${PROJECT_NAME} STATIC
  src/HoQp.cpp
  src/WbcBase.cpp
  src/HierarchicalWbc.cpp
  src/MegadogWbcRuntime.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${MEGADOG_WBC_DEPENDS} pinocchio)
target_link_libraries(${PROJECT_NAME} PUBLIC qpoases)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} qpoases
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${MEGADOG_WBC_DEPENDS} pinocchio)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(megadog_legged_interface REQUIRED)
  ament_add_gtest(test_hierarchical_wbc test/test_hierarchical_wbc.cpp)
  if(TARGET test_hierarchical_wbc)
    ament_target_dependencies(test_hierarchical_wbc ${MEGADOG_WBC_DEPENDS} pinocchio megadog_legged_interface)
    target_link_libraries(test_hierarchical_wbc ${PROJECT_NAME})
  endif()
endif()

ament_package()
