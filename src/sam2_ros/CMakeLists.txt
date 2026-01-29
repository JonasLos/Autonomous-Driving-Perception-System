cmake_minimum_required(VERSION 3.0.2)
project(sam2_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
cmake_minimum_required(VERSION 3.8)
project(sam2_ros)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

## Generate ROS interfaces from msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedRoadArea.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()

# Install Python node scripts
install(PROGRAMS
  "src/samv2_image_segmenation.py"
  "src/samv2_mask_transform.py"
  DESTINATION lib/${PROJECT_NAME}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
