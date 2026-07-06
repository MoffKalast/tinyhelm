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
  tinyhelm_core
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs dynamic_reconfigure rospy move_base_msgs nav_msgs sensor_msgs std_msgs laser_geometry tf2_ros tf2_sensor_msgs tinyhelm_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
add_dependencies(scan_to_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_cloud ${catkin_LIBRARIES})

add_executable(obstacle_planner_node src/obstacle_planner_node.cpp)
add_dependencies(obstacle_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_planner_node ${catkin_LIBRARIES})

add_executable(collision_monitor src/collision_monitor.cpp)
add_dependencies(collision_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_monitor ${catkin_LIBRARIES})
