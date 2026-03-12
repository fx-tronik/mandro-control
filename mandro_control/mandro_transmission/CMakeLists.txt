cmake_minimum_required(VERSION 2.8.3)
project(mandro_transmission)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hardware_interface
  pluginlib
  roscpp
  transmission_interface
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS transmission_interface
)

# Include header-only Boost libraries
find_package(Boost REQUIRED)

# Include a custom cmake file for TinyXML
find_package(TinyXML REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mandro_transmission src/rack_and_pinion_loader.cpp src/lead_screw_loader.cpp src/gripper_loader.cpp src/c1c2_transmission_loader.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

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

## Install plugins
install(FILES mandro_transmission_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)