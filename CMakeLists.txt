cmake_minimum_required(VERSION 3.8)
project(LQR_ros2_node_project)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


#install(FILES
#  bridge.yaml
#  params.yaml
#  rviz_config.rviz
#  DESTINATION share/${PROJECT_NAME}
#)
#
#install(DIRECTORY simulator/
#  DESTINATION share/${PROJECT_NAME}/simulator
#)


# Aggiungi l'eseguibile per il nodo csv_publisher
add_executable(csv_publisher src/csv_publisher.cpp)
ament_target_dependencies(csv_publisher rclcpp geometry_msgs)


# Installa il nodo
install(TARGETS csv_publisher
 DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 # the following line skips the linter which checks for copyrights
 # comment the line when a copyright and license is added to all source files
 set(ament_cmake_copyright_FOUND TRUE)
 # the following line skips cpplint (only works in a git repo)
 # comment the line when this package is in a git repo and when
 # a copyright and license is added to all source files
 set(ament_cmake_cpplint_FOUND TRUE)
 ament_lint_auto_find_test_dependencies()
endif()


ament_package()
