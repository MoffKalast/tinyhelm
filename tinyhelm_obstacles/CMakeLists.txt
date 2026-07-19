cmake_minimum_required(VERSION 3.0.2)
project(tinyhelm_obstacles)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  dynamic_reconfigure
  rospy
  move_base_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  laser_geometry
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tinyhelm_core
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs dynamic_reconfigure rospy move_base_msgs nav_msgs sensor_msgs std_msgs laser_geometry tf2_ros tf2_sensor_msgs tf2_geometry_msgs tinyhelm_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
add_dependencies(scan_to_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_cloud ${catkin_LIBRARIES})

install(PROGRAMS
  scripts/obstacle_planner_node.py
  scripts/collision_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/chunked_grid.py
  scripts/cost_field.py
  scripts/theta_star.py
  scripts/corridor_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
