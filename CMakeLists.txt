cmake_minimum_required(VERSION 3.5)
project(odom_frame_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(odom_frame_publisher_component SHARED
  src/odom_frame_publisher_component.cpp)
target_compile_definitions(odom_frame_publisher_component
  PRIVATE "ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(odom_frame_publisher_component
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  quaternion_operation)
rclcpp_components_register_nodes(odom_frame_publisher_component
  "odom_frame_publisher::NmeaGpsDriverComponent")

add_executable(odom_frame_publisher_node
  src/odom_frame_publisher_node.cpp
)
target_link_libraries(odom_frame_publisher_node odom_frame_publisher_component)

install(TARGETS
  odom_frame_publisher_node
  DESTINATION lib/odom_frame_publisher
)
install(TARGETS odom_frame_publisher_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
ament_export_libraries(odom_frame_publisher_component)

ament_export_include_directories(include)
ament_package()