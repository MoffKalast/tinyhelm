cmake_minimum_required(VERSION 3.0.2)
project(tinyhelm_obstacles)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  dynamic_reconfigure
  message_generation
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

add_message_files(
  FILES
  PlanRequest.msg
  PlanReply.msg
  PathWatch.msg
  PathStatus.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs dynamic_reconfigure message_runtime rospy move_base_msgs nav_msgs sensor_msgs std_msgs laser_geometry tf2_ros tf2_sensor_msgs tf2_geometry_msgs tinyhelm_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
add_dependencies(scan_to_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_cloud ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/costmap_node.py
  scripts/planner_node.py
  scripts/obstacle_monitor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(tinyhelm_obstacles_PY_MODULES local_grid cost_field theta_star coarse_heuristic mission_state)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
foreach(module ${tinyhelm_obstacles_PY_MODULES})
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${module}.py
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${module}.py
  )
endforeach()

install(FILES
  scripts/local_grid.py
  scripts/cost_field.py
  scripts/theta_star.py
  scripts/coarse_heuristic.py
  scripts/mission_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)