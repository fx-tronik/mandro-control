#!/usr/bin/env python
import rospy
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
import numpy as np

rospy.init_node('camera_pose_publisher', anonymous=True)
cam_pub = rospy.Publisher('pose_camera', Pose, queue_size=10)
eff_pub = rospy.Publisher('pose_effector', Pose, queue_size=10)
tf_listener = tf.TransformListener()
    
def callback(data):
    (position, rotation) = tf_listener.lookupTransform('/base_link', '/camera', rospy.Time(0))
    p = Pose()
    p.position.x = position[0]
    p.position.y = position[1]
    p.position.z = position[2]
    p.orientation.x = rotation[0]
    p.orientation.y = rotation[1]
    p.orientation.z = rotation[2]
    p.orientation.w = rotation[3]
    cam_pub.publish(p)
    
    (position, rotation) = tf_listener.lookupTransform('/base_link', '/effector', rospy.Time(0))
    p = Pose()
    p.position.x = position[0]
    p.position.y = position[1]
    p.position.z = position[2]
    rpy = tf.transformations.euler_from_quaternion(rotation)
    p.orientation.x = np.rad2deg(rpy[0])
    p.orientation.y = np.rad2deg(rpy[1])
    p.orientation.z = np.rad2deg(rpy[2])
    p.orientation.w = rotation[3]
    eff_pub.publish(p)

if __name__ == '__main__':
    sub = rospy.Subscriber("tf", TFMessage, callback)
    rospy.spin() 
