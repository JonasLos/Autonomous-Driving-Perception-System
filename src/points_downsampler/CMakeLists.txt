cmake_minimum_required(VERSION 3.5)
project(points_downsampler)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(autoware_config_msgs REQUIRED)
find_package(velodyne_pcl REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointsDownsamplerInfo.msg"
  DEPENDENCIES std_msgs
)
include_directories(include)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
include_directories(include)
add_compile_options(-O2 -g -Wall)

add_executable(voxel_grid_filter nodes/voxel_grid_filter/voxel_grid_filter.cpp)
ament_target_dependencies(voxel_grid_filter rclcpp sensor_msgs pcl_conversions autoware_config_msgs velodyne_pcl)
target_link_libraries(voxel_grid_filter ${PCL_LIBRARIES})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(voxel_grid_filter ${cpp_typesupport_target})

add_executable(ring_filter nodes/ring_filter/ring_filter.cpp)
ament_target_dependencies(ring_filter rclcpp sensor_msgs pcl_conversions autoware_config_msgs velodyne_pcl)
target_link_libraries(ring_filter ${PCL_LIBRARIES})
target_link_libraries(ring_filter ${cpp_typesupport_target})

add_executable(distance_filter nodes/distance_filter/distance_filter.cpp)
ament_target_dependencies(distance_filter rclcpp sensor_msgs pcl_conversions autoware_config_msgs)
target_link_libraries(distance_filter ${PCL_LIBRARIES})
target_link_libraries(distance_filter ${cpp_typesupport_target})

add_executable(random_filter nodes/random_filter/random_filter.cpp)
ament_target_dependencies(random_filter rclcpp sensor_msgs pcl_conversions autoware_config_msgs)
target_link_libraries(random_filter ${PCL_LIBRARIES})
target_link_libraries(random_filter ${cpp_typesupport_target})

install(TARGETS
  voxel_grid_filter
  ring_filter
  distance_filter
  random_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
