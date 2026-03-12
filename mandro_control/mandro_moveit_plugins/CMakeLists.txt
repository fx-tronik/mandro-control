cmake_minimum_required(VERSION 2.8.3)
project(mandro_moveit_plugins)

## Compile as C++17, supported in ROS Melodic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  controller_manager_msgs
  moveit_core
  moveit_simple_controller_manager
  pluginlib
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_core controller_manager_msgs trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mandro_moveit_plugins src/gripper_action_controller_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES mandro_moveit_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)