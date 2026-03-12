cmake_minimum_required(VERSION 2.8.3)

project(scara_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
)

## Generate actions in the 'action' folder
 add_action_files(FILES
   robot_control.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(DEPENDENCIES
   actionlib_msgs
)
catkin_package(CATKIN_DEPENDS 
   actionlib_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
