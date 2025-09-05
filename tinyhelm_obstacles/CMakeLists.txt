cmake_minimum_required(VERSION 3.0.2)
project(tinyhelm_obstacles)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  dynamic_reconfigure
  rospy
  move_base_msgs
  std_msgs
  laser_geometry
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs dynamic_reconfigure rospy move_base_msgs std_msgs laser_geometry tf2_ros tf2_sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/sensor_to_obstacle_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
add_dependencies(scan_to_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_cloud ${catkin_LIBRARIES})


add_executable(scan_to_grid src/scan_to_grid.cpp)
add_dependencies(scan_to_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_grid ${catkin_LIBRARIES})