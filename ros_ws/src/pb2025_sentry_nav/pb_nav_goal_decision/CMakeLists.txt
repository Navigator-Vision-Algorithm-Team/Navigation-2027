cmake_minimum_required(VERSION 3.8)
project(pb_nav_goal_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# 本发行版(lyrical)的 ament_cmake 不会自动引入 ament_target_dependencies，
# 需显式 find 这个拆出来的包（标准 ROS 发行版通常不需要）。
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Build as a composable component (also yields a standalone executable).
add_library(${PROJECT_NAME} SHARED
  src/goal_decision_node.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  tf2
  ament_index_cpp
  yaml-cpp
  nav2_msgs
  std_msgs)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "pb_nav_goal_decision::GoalDecisionNode"
  EXECUTABLE goal_decision_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
