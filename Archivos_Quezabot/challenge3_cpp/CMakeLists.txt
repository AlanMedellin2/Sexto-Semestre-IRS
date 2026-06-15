cmake_minimum_required(VERSION 3.8)
project(challenge3_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies "rclcpp" "geometry_msgs" "nav_msgs" "std_msgs")

install(DIRECTORY
  launch 
  config 
  src 
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables #They must start with the shebag tag
install(PROGRAMS
  scripts/node_odometry.py
  scripts/node_extra.py
  scripts/node_extra_challenge3.py
  scripts/error.py
  scripts/error_challenge3.py
  scripts/closed_loop_ctrl.py
  scripts/rotate_node.py
  scripts/color_detector_node.py
  DESTINATION lib/${PROJECT_NAME} )

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

