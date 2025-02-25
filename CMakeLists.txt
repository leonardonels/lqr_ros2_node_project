cmake_minimum_required(VERSION 3.5)
project(LQR_ros2_node_project)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(lqr_node src/lqr_node.cpp)
ament_target_dependencies(lqr_node rclcpp nav_msgs)

install(TARGETS
  lqr_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
