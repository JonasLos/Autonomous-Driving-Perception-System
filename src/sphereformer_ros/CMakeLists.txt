cmake_minimum_required(VERSION 3.0.2)
project(sphereformer_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
cmake_minimum_required(VERSION 3.8)
project(sphereformer_ros)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

# Install Python node scripts
install(PROGRAMS
  "src/sphereformer_ros.py"
  "src/semantic_kitti_ros.py"
  DESTINATION lib/${PROJECT_NAME}
)

