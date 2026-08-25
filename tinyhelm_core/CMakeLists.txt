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

add_executable(cmd_vel_mux src/cmd_vel_mux.cpp)
add_dependencies(cmd_vel_mux ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_mux ${catkin_LIBRARIES})

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
