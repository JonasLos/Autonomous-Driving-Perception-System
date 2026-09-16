cmake_minimum_required(VERSION 3.8)
project(fusion_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Order is fixed deliberately: a message's type hash depends on its own definition and its
# dependencies, and keeping a stable declaration order keeps diffs readable.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection3D.msg"
  "msg/Detection3DArray.msg"
  "msg/FusedObject.msg"
  "msg/FusedObjectArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
