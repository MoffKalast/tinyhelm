cmake_minimum_required(VERSION 3.0.2)
project(tinyhelm_stationkeeping)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  dynamic_reconfigure
  rospy
  move_base_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  tinyhelm_core
)

generate_dynamic_reconfigure_options(
  config/StationKeeping.cfg
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs dynamic_reconfigure rospy move_base_msgs std_msgs nav_msgs visualization_msgs tf tf2_ros tf2_geometry_msgs tinyhelm_core message_runtime
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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
