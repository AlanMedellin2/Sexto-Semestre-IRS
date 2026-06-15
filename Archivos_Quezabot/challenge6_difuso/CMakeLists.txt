cmake_minimum_required(VERSION 3.8)
project(challenge6_difuso)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  scripts/line_detector.py
  scripts/yolo_decision_node.py
  scripts/fuzzy_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
