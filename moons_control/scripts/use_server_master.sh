#!/usr/bin/env sh
export ROS_HOSTNAME=$(/bin/hostname -I)
export ROS_MASTER_URI=http://192.168.0.250:11311
roslaunch moons_control client.launch