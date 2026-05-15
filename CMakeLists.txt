cmake_minimum_required(VERSION 3.8)
project(scanner)

# -----------------------------
# Find dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

# -----------------------------
# Build node
# -----------------------------
add_executable(scanner_node
  src/scanner_node.cpp
)

ament_target_dependencies(scanner_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib_msgs
)

# -----------------------------
# Install
# -----------------------------
install(TARGETS scanner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
