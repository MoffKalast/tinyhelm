cmake_minimum_required(VERSION 3.0.2)
project(tinyhelm_stationkeeping)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  dynamic_reconfigure
  rospy
  move_base_msgs
  std_msgs
  message_generation
)

generate_dynamic_reconfigure_options(
  config/StationKeeping.cfg
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs dynamic_reconfigure rospy move_base_msgs std_msgs message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    scripts/stationkeeping_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
