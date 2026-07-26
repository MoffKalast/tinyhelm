cmake_minimum_required(VERSION 3.0.2)
project(tinyhelm_obstacles)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  rospy
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
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
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime rospy nav_msgs sensor_msgs std_msgs visualization_msgs laser_geometry tf2_ros tf2_sensor_msgs tf2_geometry_msgs tinyhelm_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
add_dependencies(scan_to_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_cloud ${catkin_LIBRARIES})

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