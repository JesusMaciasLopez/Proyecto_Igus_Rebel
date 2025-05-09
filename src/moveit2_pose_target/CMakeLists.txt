cmake_minimum_required(VERSION 3.8)
project(moveit2_pose_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(move_to_pose_node src/move_to_pose_node.cpp)

ament_target_dependencies(
  move_to_pose_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  sensor_msgs  
)

install(TARGETS move_to_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
