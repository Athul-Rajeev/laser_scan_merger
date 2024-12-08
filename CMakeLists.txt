cmake_minimum_required(VERSION 3.0.2)
project(laser_scan_merger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_geometry
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
  message_filters
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ScanMerger.cfg
)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs pcl_ros tf2_ros message_filters dynamic_reconfigure
)

include_directories(
  include
  config
  include/laser_scan_merger
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_scan_merger_node src/LaserScanMerger.cpp)
add_dependencies(laser_scan_merger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_scan_merger_node ${catkin_LIBRARIES})
