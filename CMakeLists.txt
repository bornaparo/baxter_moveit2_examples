cmake_minimum_required(VERSION 3.8)
project(baxter_moveit2_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)

# find_package(actionlib REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(roscpp REQUIRED)

find_package(control_msgs REQUIRED)

find_package(rviz_gazebo_bridge REQUIRED)

add_executable(baxter_moveit2_examples src/baxter_moveit2_examples.cpp)
ament_target_dependencies(baxter_moveit2_examples moveit_ros_planning_interface ignition-msgs8 ignition-transport11 rclcpp moveit_msgs tf2_geometry_msgs geometric_shapes control_msgs rviz_gazebo_bridge)

install(TARGETS 
  baxter_moveit2_examples
  DESTINATION lib/${PROJECT_NAME}
)

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

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
