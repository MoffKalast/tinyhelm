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

file(GLOB PYTHON_NODES "scripts/*_node.py")
file(GLOB PYTHON_MODULES "scripts/*.py")

list(REMOVE_ITEM PYTHON_MODULES ${PYTHON_NODES})

catkin_install_python(
  PROGRAMS ${PYTHON_NODES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(module ${PYTHON_MODULES})
  get_filename_component(name ${module} NAME)
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${module}
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${name}
  )
endforeach()

install(
  FILES ${PYTHON_MODULES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY vessel_config world_config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)