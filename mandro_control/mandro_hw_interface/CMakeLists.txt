cmake_minimum_required(VERSION 2.8.3)
project(mandro_hw_interface)

## Compile as C++17, supported in ROS Melodic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS 
  moons_servo 
  ethercat_controller 
  joint_limits_interface 
  hardware_interface 
  transmission_interface 
  rosparam_shortcuts 
  urdf 
  mandro_transmission 
  std_msgs 
  message_generation
)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES estop.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mandro_hw_interface
  CATKIN_DEPENDS moons_servo ethercat_controller transmission_interface mandro_transmission rosparam_shortcuts urdf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/mandro_hardware_interface.cpp 
  src/joints/common_joint.cpp 
  src/joints/ec_joint.cpp 
  src/joints/dummy_joint.cpp 
  src/joints/mimic_joint.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)