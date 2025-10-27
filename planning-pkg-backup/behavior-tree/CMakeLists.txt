cmake_minimum_required(VERSION 3.8)
project(behavior-tree)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(run_tree src/run_tree.cpp)
ament_target_dependencies(run_tree rclcpp behaviortree_cpp ament_index_cpp)

install(TARGETS run_tree
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

ament_package()
