cmake_minimum_required(VERSION 2.8.3)
project(moons_control)

find_package(catkin REQUIRED COMPONENTS controller_manager diagnostic_updater transmission_interface hardware_interface sensor_msgs joint_limits_interface trajectory_msgs soem realtime_tools rosparam_shortcuts)

#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-9")
#set(CMAKE_C_COMPILER "/usr/bin/clang-9")
add_compile_options(-std=c++17)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS diagnostic_updater trajectory_msgs sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ecat_controller moons_transmission
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ## The following work around allows SOEM headers to include other SOEM headers.
  ## SOEM headers assume all headers are installed in a flat directory structure
  ## See https://github.com/smits/soem/issues/4 for more information.
  ${soem_INCLUDE_DIRS}/soem
)

## Declare a C++ library
add_library(servo_client src/servo_client.cpp)
add_library(ecat_controller src/ecat_controller.cpp)
add_library(moons_transmission src/transmissions/c1c2_transmission_loader.cpp src/transmissions/gripper_transmission_loader.cpp)

## Declare a C++ executable for ROS_CONTROL
add_executable(homing src/homing.cpp)
add_executable(main src/main.cpp src/moons_hardware_interface.cpp src/joints/common_joint.cpp src/joints/ec_joint.cpp src/joints/dummy_joint.cpp src/joints/mimic_joint.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ecat_controller ${catkin_LIBRARIES} )
target_link_libraries(servo_client ${catkin_LIBRARIES} ecat_controller)
target_link_libraries(main ${catkin_LIBRARIES} servo_client)
target_link_libraries(homing ${catkin_LIBRARIES} servo_client)

function(set_sudo_binary tgt)
  get_target_property(loc ${tgt} "LOCATION")
  string(ASCII 27 ESCAPE)
  add_custom_target(sudo_${tgt}
    COMMAND sudo -n sh -c 'setcap CAP_SYS_NICE,cap_net_raw,cap_ipc_lock=+ep ${loc}' || (>&2 echo "${ESCAPE}[31m#\ type\ following\ command\ before\ execute\ reset...\ sudo\ setcap cap_net_raw,cap_ipc_lock=+ep\ ${loc}${ESCAPE}[0m")
    DEPENDS bin/${tgt})
  add_custom_target(sudo_${tgt}_all ALL DEPENDS sudo_${tgt})
endfunction()

set_sudo_binary(main)
set_sudo_binary(homing)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_moons_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
