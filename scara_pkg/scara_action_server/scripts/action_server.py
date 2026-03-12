#! /usr/bin/env python
# coding=utf-8

import rospy
import actionlib
import sys
import moveit_commander
import scara_action_server.msg as action_server_msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from std_msgs.msg import String, Header
from scipy import interpolate
import tf
import numpy as np
import time
import mandro_io

USE_IK = True

def position_to_pose(xyz, rpy):
    roll  = np.deg2rad(rpy[0])*0    # przegub dodatkowy - nieobslugiwany - zawsze 0
    pitch = np.deg2rad(rpy[1])		# kat obrotu chwytaka (wyrywanie), zakres -50:50 stopni
    yaw   = np.deg2rad(rpy[2])		# kat obrotu chwytaka (w osi Z),  zakres -140:140 stopni, w zaleznosci od pozycji
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    position = geometry_msgs.msg.Point(*(np.array(xyz)*0.001))
    orientation = geometry_msgs.msg.Quaternion(*quaternion)
    pose = geometry_msgs.msg.Pose(position, orientation)
    return pose

def start_state(pose):
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint7']
    joint_state.position = pose
    start_state = RobotState()
    start_state.joint_state = joint_state
    return start_state

def add_gripper_to_plan(plan, gripper_start_position, points_ids, gripper_positions):
    if len(points_ids) != len(gripper_positions) or len(points_ids) < 1:
        return
    plan.joint_trajectory.joint_names.extend(['joint6'])
    len_points = len(plan.joint_trajectory.points)
    points_ids = list(points_ids) + [len_points]
    
    joint6 = np.full((len_points), gripper_start_position, dtype=np.float)
    gripper_positions = np.array(gripper_positions) * 0.0005
    
    for first, last, gp in zip(points_ids[:-1], points_ids[1:], gripper_positions):
        joint6[first:last] = gp
    joint6[0] = gripper_start_position

    for i in range(len_points):
        plan.joint_trajectory.points[i].positions = list(plan.joint_trajectory.points[i].positions) + [joint6[i]]
        plan.joint_trajectory.points[i].velocities = list(plan.joint_trajectory.points[i].velocities) + [0]
        plan.joint_trajectory.points[i].accelerations = list(plan.joint_trajectory.points[i].accelerations) + [0]     

def smooth_corner(points, corner_length, count):
    if len(points) != 3:
        return None
    
    npoints = np.array(points, dtype=np.float)
    v = npoints[1] - npoints[0]
    vn = np.linalg.norm(v)
    npoints[0] = npoints[1] - corner_length/vn * v
    v = npoints[2] - npoints[1]
    vn = np.linalg.norm(v)
    npoints[2] = npoints[1] + corner_length/vn * v
    
    mid = np.mean(npoints[[0,2]], axis=0)
    new_corner = np.mean([npoints[1], mid], axis=0)
    npoints[1] = new_corner
    
    tck, u = interpolate.splprep([npoints[:,0], npoints[:,1], npoints[:,2]], s=0, k=2)
    x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
    u_fine = np.linspace(0,1,count)
    x, y, z = interpolate.splev(u_fine, tck)
    interpolated_points = np.column_stack((x,y,z))
    #output_points = np.vstack((points[0], interpolated_points, points[-1]))
    return interpolated_points

class MoveGroupInterface():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.tolerance = 0.001
        self.step = 0.01
        self.jump_threshold = 10
        self.cartesianVel = 1.0
        self.cartesianAcc = 1.0
        self.cartesianAlgorithm = "time_optimal_trajectory_generation"
        self.moveGroupVel = 0.5
        self.moveGroupAcc = 0.5
        
        self.cutting_pos 	    = [1100, -450, 50]
        self.cutting_orient	    = [0, 0, -130]

        self.box_pos 		    = [1000, -250, 120]
        self.box_orient		    = [0, 0, -110]

        self.place_plan		    = 0
        self.place_last_point   = 0	
        self.place_last_time    = 0
        self.plan_points_len    = 0
        self.times = []
    
        self.robot               = moveit_commander.RobotCommander()
        self.scene               = moveit_commander.PlanningSceneInterface()
        self.move_group_arm      = moveit_commander.MoveGroupCommander('arm')
        self.move_group_gripper  = moveit_commander.MoveGroupCommander('gripper')
        self.move_group_arm.set_goal_tolerance(self.tolerance)
        self.move_group_gripper.set_goal_tolerance(self.tolerance)
    
        self.move_group_arm.set_max_velocity_scaling_factor(self.moveGroupVel)
        self.move_group_arm.set_max_acceleration_scaling_factor(self.moveGroupAcc)
    
        self._as = ActionServer(rospy.get_name(), self)
        self.io = mandro_io.IO()

    def set_gripper_width(self, width, wait=False):
        return self.move_group_gripper.go([width*0.001/2, -width*0.001/2], wait=wait)

    def set_joints(self, joint_states):
        # joints_state = [joint1, joint2, joint3, joint4, joint5, joint6]
        # joint1 - wozek przesuw x
        # joint2 - wozek przesuw z
        # joint3 - obrót długie ramię
        # joint4 - obrót krótkie ramię
        # joint5 - przechylenie chwytaka w osi y - ruch wyrywający
        # joint6 - chwytak - rozstaw łapek
        # joint7 - fake_joint - obrót chwytaka w osi x
        js = list(joint_states)
        joints = js[:5] + [0] + [js[5]] + [2*js[5]]
        # passive and fake joints appended
        # joints = [joint1, joint2, joint3, joint4, joint5, joint7, joint6, joint6_mimic]
        self.move_group_arm.go(joints[:6], wait=True)
        self.move_group_gripper.go(joints[6:8], wait=True)
        pose = self.move_group_arm.get_current_pose().pose
        position = [pose.position.x*1000, pose.position.y*1000, pose.position.z*1000]
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        rpy = [np.rad2deg(rpy[0]), np.rad2deg(rpy[1]), np.rad2deg(rpy[2])]
        self._as.publish_feedback({'position': position + rpy,
                                   'joints': js,
                                   'progress': 99.0})
        # TODO - zwracanie statusu wykonanej operacji
        return True

    def go_to_position(self, xyz, rpy):
        pose_goal = position_to_pose(xyz, rpy)
        if USE_IK:
            try: 
                self.move_group_arm.set_joint_value_target(pose_goal)
                joint_states = self.move_group_arm.get_joint_value_target()
                res = self.move_group_arm.go(joint_states, wait=True)
            except Exception as e:
                rospy.logerr(e)
                rospy.logerr("IK solver failed")
                joint_states = [0,0,0,0,0,0]
                res = False
        else:
            self.move_group_arm.set_pose_target(pose_goal)
            res = self.move_group_arm.go(wait=True)
        self._as.publish_feedback({'position': xyz + rpy, 'joints': joint_states, 'progress': 99.0})
        return res

    def pick(self, xyz, grip_width, pitch_angle, grip_angle, h_start, h_end, pull_length, width_closed):
        start = time.time()
        current_pose = self.move_group_arm.get_current_pose().pose
        current_position = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        arm_joint_state = start_state(self.move_group_arm.get_current_joint_values())
        self.move_group_arm.set_start_state(arm_joint_state)
        
        resolution = 10
        z_angle = np.deg2rad(-grip_angle)
        y_angles = np.linspace(0, np.deg2rad(-pitch_angle), resolution)
        points_z = np.reshape(np.linspace(-h_start, -h_end, resolution), (-1,1))
        points_xy = np.zeros((resolution,2))
        points = np.hstack((points_xy, points_z))
        
        if pull_length >= 1:
            points = np.append(points, [[0, 0, -h_end - pull_length]], axis=0)
            y_angles = np.append(y_angles, y_angles[-1])
        pitch_angles = y_angles*np.rad2deg(-1)
            
        curve = []
        for y_angle, point in zip(y_angles, points):
            matrix = tf.transformations.euler_matrix(z_angle, y_angle, 0, 'szyx')
            point_rotated = np.dot(point, matrix[:3,:3])
            curve.append(point_rotated-points[0])
        
        corner_points = [current_position*1000 - np.array(xyz),
                         [0, 0, -xyz[2]],
                         [0, 0, 0]]

        corner = smooth_corner(corner_points, 1, 11)
        start_corner = np.vstack((corner_points[0], corner))
        diffs = np.diff(start_corner, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        dists_sum = [dists[:i+1].sum() for i in range(len(dists))]
        length = dists_sum[-1]
        
        # all 6.24
        # no angle smooth 6.21
        # smooth_corner 1, 21 -> 6.25
        corner = np.append(corner, np.zeros((len(corner), 3)), axis=1)
        #corner[:,-1] = [-grip_angle * (1 - d/length) for d in dists_sum]
        rospy.logwarn(grip_angle)
        rospy.logwarn(corner[:,-1] + grip_angle)
        
        wpt = np.zeros((1+len(curve), 6), dtype=np.float)
        wpt[0:-1,:3] = np.array(curve)
        wpt[0:-1, 4] = pitch_angles
        wpt[-1, :] = [0, 0, -xyz[2], 0, 0, 0]
        wpt = np.vstack((corner, wpt))
        
        base = np.array([xyz[0], xyz[1], xyz[2], 0, 0, grip_angle])
        waypoints = [position_to_pose(base[:3]+w[:3], base[3:]+w[3:]) for w in wpt]
        plan, f = self.move_group_arm.compute_cartesian_path(waypoints, self.step, self.jump_threshold)
    
        if f != 1.0:
            waypoints = [position_to_pose(base[:3]+w[:3], (base[3:]+w[3:]+[0,0,180])*[1,-1,1]) for w in wpt]
            plan, f = self.move_group_arm.compute_cartesian_path(waypoints, self.step, self.jump_threshold)
            if f != 1.0:
                return False
            
        robot_state = self.robot.get_current_state()
        plan = self.move_group_arm.retime_trajectory(robot_state, plan, 
                                                     self.cartesianVel, self.cartesianAcc, self.cartesianAlgorithm)
        
        for i, pt in enumerate(plan.joint_trajectory.points):
            if -pt.positions[1] > xyz[2]*0.001:
                add_gripper_to_plan(plan, robot_state.joint_state.position[6], [0, i], [grip_width, 30])
                break
        
        plan_len = len(plan.joint_trajectory.points)
        duration = plan.joint_trajectory.points[-1].time_from_start
        duration_ms = duration.secs*1E3+duration.nsecs/1E6
        self._as.publish_feedback({'points_count': plan_len, 'eta': duration_ms, 'progress': 0.0})
    
        res = self.move_group_arm.execute(plan, wait=True)
        self.times.append(time.time()-start)
        rospy.logwarn('{:.2f} {:.2f} {:.2f}'.format(np.min(self.times), np.mean(self.times), np.max(self.times)))
        return res

    def first_trajectory(self, start_pose, goal_pos, goal_orient):
        state = start_state(start_pose)
        self.move_group_arm.set_start_state(state)
        pose_goal = position_to_pose(goal_pos, goal_orient)
        self.move_group_arm.set_joint_value_target(pose_goal)
        self.place_plan = self.move_group_arm.plan()
        
        self.plan_points_len  = len(self.place_plan.joint_trajectory.points)
        self.place_last_point = self.place_plan.joint_trajectory.points[self.plan_points_len-1].positions
        self.place_last_time  = self.place_plan.joint_trajectory.points[self.plan_points_len-1].time_from_start

    def next_trajectory(self, start_pose, goal_pos, goal_orient):
        state = start_state(start_pose)
        self.move_group_arm.set_start_state(state)
        goal_pose = position_to_pose(goal_pos, goal_orient)
        self.move_group_arm.set_joint_value_target(goal_pose)
        plan = self.move_group_arm.plan().joint_trajectory.points
        self.plan_points_len = len(plan)
        
        for i in range(self.plan_points_len):
            plan[i].time_from_start += self.place_last_time
        
        self.place_plan.joint_trajectory.points.extend(plan[1:])
        
        self.place_last_point = plan[self.plan_points_len-1].positions
        self.place_last_time  = plan[self.plan_points_len-1].time_from_start
        
    def place(self, end_pos, end_orient):
# =============================================================================
#         points = [self.cutting_pos[:2] + [0],
# 				  self.cutting_pos,
# 				  self.box_pos[:2] + [40],
# 				  self.box_pos]
# 
#         points = smooth_path(points, 0, 20)
#         self.first_trajectory(points[0],  self.cutting_orient)
#         
#         for i, pt in enumerate(points[1:]):
#             self.next_trajectory (self.place_last_point, pt, self.cutting_orient)
#         # if i == len(points)-2:
#         # 	open_gripper = len(self.place_plan.joint_trajectory.points)
# =============================================================================
        
        
# =============================================================================
#         positions = [self.cutting_pos[:2] + [0],
#                      self.cutting_pos,
#                      self.cutting_pos[:2] + [40],
#                      self.box_pos[:2] + [40],
#                      self.box_pos,
#                      self.box_pos[:2] + [0]]
#         orients = [self.cutting_orient,
#                    self.cutting_orient,
#                    self.cutting_orient,
#                    self.box_orient,
#                    self.box_orient,
#                    self.box_orient]
#         
#         pose = position_to_pose(positions[0], orients[0])
#         self.move_group_arm.go(pose, wait=True)
#         arm_joint_state = start_state(self.move_group_arm.get_current_joint_values())
#         self.move_group_arm.set_start_state(arm_joint_state)
#         waypoints = [position_to_pose(xyz, rpy) for xyz, rpy in zip(positions[1:], orients[1:])]
#         plan, f = self.move_group_arm.compute_cartesian_path(waypoints, self.step, self.jump_threshold)
#             
#         robot_state = self.robot.get_current_state()
#         plan = self.move_group_arm.retime_trajectory(robot_state, plan, 
#                                                      self.cartesianVel, self.cartesianAcc, self.cartesianAlgorithm)
# 
#         self.io.cutter_on()
#         res = self.move_group_arm.execute(plan, wait=True)
#         self.io.cutter_off()
#         return res
# =============================================================================
        
        
        start_pose = self.move_group_arm.get_current_joint_values()
        self.first_trajectory(start_pose,			 self.cutting_pos[:2] + [0],  self.cutting_orient)
        self.next_trajectory (self.place_last_point, self.cutting_pos, 			  self.cutting_orient)
        self.next_trajectory (self.place_last_point, self.cutting_pos[:2] + [40], self.cutting_orient)
        
        self.next_trajectory (self.place_last_point, self.box_pos[:2] + [40], 	  self.box_orient)
        self.next_trajectory (self.place_last_point, self.box_pos, 			 	  self.box_orient)
        
        open_gripper = len(self.place_plan.joint_trajectory.points)
        self.next_trajectory (self.place_last_point, self.box_pos[:2] + [0], 	  self.box_orient)
        #self.next_trajectory (self.place_last_point, end_pos, 	 				  end_orient)
        
        add_gripper_to_plan(self.place_plan, self.move_group_gripper.get_current_joint_values()[0], 
                            [open_gripper], [65])
        self.io.cutter_on()
        res = self.move_group_arm.execute(self.place_plan)
        self.io.cutter_off()
        return res

class ActionServer(object):
    def __init__(self, name, robot):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action_server_msg.robot_controlAction,
                                                execute_cb=self.execute_cb, auto_start=False)

        self._feedback = action_server_msg.robot_controlFeedback()
        self._result = action_server_msg.robot_controlResult()
        self._as.start()
        self._robot = robot
        
    def publish_feedback(self, values):
        for key in values:
            if hasattr(self._feedback, key):
                if type(values[key] == type(getattr(self._feedback, key))):
                    setattr(self._feedback, key, values[key])
                else:
                    rospy.logwarn('ActionServer - wrong feedback key type')
            else:
                rospy.logwarn('ActionServer - wrong feedback key name')
        self._as.publish_feedback(self._feedback)

    def execute_cb(self, goal):
        position = goal.position
        command = goal.command.data

        self.publish_feedback({'command': String(command), 'eta': 0, 'points_count': 0, 
                               'progress': 0.1, 'position': [], 'joints': []})
        
        rospy.logwarn(command.upper())
        if command == 'pick':
            # [x, y, z, grip_width, pitch_angle, grip_angle, h_start, h_end, pull_length, width_closed]
            grip_width, pitch_angle, grip_angle = position[3:6]
            h_start = position[6] if len(position) >= 7 else 50
            h_end = position[7] if len(position) >= 8 else h_start
            pull_length = position[8] if len(position) >= 9 else 0
            width_closed = position[9] if len(position) >= 10 else 30
            self._result.success = self._robot.pick(position[:3], grip_width, pitch_angle, \
                                                    grip_angle, h_start, h_end, pull_length, width_closed)
        elif command == 'place':
            self._result.success = self._robot.place([-300,750,0],[0,0,0])
# =============================================================================
#             success_arm = self._robot.go_to_position(position[:3], position[3:])
#             success_gripper = self._robot.set_gripper_width(65, wait=True)
#             self._result.success = success_arm and success_gripper
# =============================================================================
        elif command == 'goto':
            self._result.success = self._robot.go_to_position(position[:3], position[3:])
        elif command == 'home':
            self._result.success = self._robot.set_joints([0, 0, 0, 0, 0, 0])
        elif command == 'set_joints':
            self._result.success = self._robot.set_joints(position)
        else:
            self._result.success = False

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('robot_action_server')
    time.sleep(5)
    Robot = MoveGroupInterface()
    rospy.spin()
