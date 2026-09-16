cmake_minimum_required(VERSION 3.8)
project(delphi_esr_driver)

# Interfaces only -- see package.xml. The upstream CMakeLists also builds
# delphi_esr_driver_node via ament_auto_find_build_dependencies(), which requires
# can_msgs, kvaser_interface and raptor_dbw_msgs. None of those belong in a perception
# image, and the containers never run the driver: they only subscribe to its output.
#
# The msg list and DEPENDENCIES here must match upstream exactly, or the generated type
# hash diverges and the subscription silently stops matching the live driver.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/EsrTrack.msg
  msg/EsrTrackArray.msg
  msg/EsrStatus.msg
  DEPENDENCIES std_msgs
)

ament_package()
