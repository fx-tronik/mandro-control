cmake_minimum_required(VERSION 2.8.3)
project(moons_servo)

## Compile as C++17, supported in ROS Melodic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ethercat_controller 
  joint_limits_interface 
  std_msgs 
  message_generation
)

add_service_files(
  FILES home.srv reset.srv go2pos.srv estop.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moons_servo
  CATKIN_DEPENDS ethercat_controller
  #DEPENDS moons_control
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
add_library(${PROJECT_NAME} src/client.cpp)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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