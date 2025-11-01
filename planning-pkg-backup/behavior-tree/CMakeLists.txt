cmake_minimum_required(VERSION 3.8)
project(behavior-tree)

# Use modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------------------------------
# Dependencies
# -------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

# -------------------------------------------------
# Test node (stand-alone)
# -------------------------------------------------
add_executable(test_rclcpp_node src/test_rclcpp_node.cpp)
ament_target_dependencies(test_rclcpp_node rclcpp)
install(TARGETS test_rclcpp_node DESTINATION lib/${PROJECT_NAME})

# -------------------------------------------------
# Behavior-Tree executable
# -------------------------------------------------
add_executable(run_tree
  src/run_tree.cpp
  # src/check_system.cpp
  # src/navigate_to_waypoint.cpp
  # src/detect_person.cpp
  # src/approach_person.cpp
  # src/deliver_payload.cpp
  # src/replan_search_area.cpp
  # src/report_to_base.cpp
)

ament_target_dependencies(run_tree
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  behaviortree_cpp
  ament_index_cpp
)

install(TARGETS run_tree DESTINATION lib/${PROJECT_NAME})

# -------------------------------------------------
# Install XML behavior trees
# -------------------------------------------------
install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

ament_package()
