cmake_minimum_required(VERSION 3.8)
project(clrernet_ros)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## Generate ROS interfaces from msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LanePoint.msg"
  "msg/LanePoints.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()

# Install Python node scripts so they can be run as ROS2 executables
install(PROGRAMS
  "src/clrernet_lane_detection.py"
  "src/clrernet_lane_transform.py"
  "src/inference.py"
  DESTINATION lib/${PROJECT_NAME}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
