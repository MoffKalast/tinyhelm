cmake_minimum_required(VERSION 3.0.2)
project(tinyhelm_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  dynamic_reconfigure
  rospy
  move_base_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ControllerStatus.msg
  MonitorStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs dynamic_reconfigure rospy move_base_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/helm_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(cmd_vel_mux src/cmd_vel_mux.cpp)
add_dependencies(cmd_vel_mux ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_mux ${catkin_LIBRARIES})

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
