cmake_minimum_required(VERSION 3.0.2)
project(tinyhelm_waypoints)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
    move_base_msgs
    std_msgs
    message_generation
)

generate_dynamic_reconfigure_options(
    config/Waypoints.cfg
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure rospy geometry_msgs move_base_msgs std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    scripts/waypoints_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
