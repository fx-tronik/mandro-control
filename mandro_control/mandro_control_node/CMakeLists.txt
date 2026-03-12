cmake_minimum_required(VERSION 2.8.3)
project(mandro_control_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  std_msgs 
  mandro_hw_interface
  controller_manager
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mandro_control_node
  CATKIN_DEPENDS mandro_hw_interface controller_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/mandro_control_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

function(set_sudo_binary tgt)
  get_target_property(loc ${tgt} "LOCATION")
  string(ASCII 27 ESCAPE)
  add_custom_target(sudo_${tgt}
    COMMAND sudo -n sh -c 'setcap CAP_SYS_NICE,cap_net_raw,cap_ipc_lock=+ep ${loc}' || (>&2 echo "${ESCAPE}[31m#\ type\ following\ command\ before\ execute\ reset...\ sudo\ setcap cap_net_raw,cap_ipc_lock=+ep\ ${loc}${ESCAPE}[0m")
    DEPENDS bin/${tgt})
  add_custom_target(sudo_${tgt}_all ALL DEPENDS sudo_${tgt})
endfunction()

set_sudo_binary(${PROJECT_NAME})

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
