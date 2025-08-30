cmake_minimum_required(VERSION 3.5.2)
project(tinyhelm_sim)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	geometry_msgs
	sensor_msgs
	visualization_msgs
	nav_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    src/sim.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY vessel_config world_config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)