cmake_minimum_required(VERSION 3.10)
project(hybrid_force_motion_controller LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  msg/HybridForceMotionState.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs
)

add_executable(hybrid_force_motion_node
  src/hybrid_force_motion_node.cpp
)

add_executable(cartesian_velocity_controller_node
  src/cartesian_velocity_controller_node.cpp
)

target_include_directories(hybrid_force_motion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(cartesian_velocity_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hybrid_force_motion_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  Eigen3
  orocos_kdl
  kdl_parser
  urdf
)

target_link_libraries(hybrid_force_motion_node
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
)

ament_target_dependencies(cartesian_velocity_controller_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  Eigen3
  orocos_kdl
  kdl_parser
  urdf
)

target_link_libraries(cartesian_velocity_controller_node
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hybrid_force_motion_node
  ${cpp_typesupport_target})

target_link_libraries(cartesian_velocity_controller_node
  ${cpp_typesupport_target})

install(TARGETS
  hybrid_force_motion_node
  cartesian_velocity_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds reference meshes config
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
