#! /usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import JointState
import mandro_io


class VacuumControler():
    def __init__(self):
        self.io = mandro_io.IO()
        self.io.release()
        self.grab = False
        self.gripper_subscriber = rospy.Subscriber("/joint_states", JointState, self.callback)
        
    def callback(self, data):
        if abs(data.position[5] - 0.015) < 0.001:
            if self.grab == False:
                self.io.grab()
                self.grab = True
        else:
            if self.grab:
                self.io.release()
                self.grab = False


if __name__ == '__main__':
    rospy.init_node('Vacuum controler', anonymous=True)
    sub = VacuumControler()
    rospy.spin() 