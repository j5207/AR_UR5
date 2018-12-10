#!/usr/bin/env python

from __future__ import division, print_function
import numpy as np
from math import *
from time import sleep
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import actionlib
import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3
import tf
from tf.transformations import *
from math import pi
# from icl_phri_robotiq_control.robotiq_utils import *
from inverseKinematicsUR5 import InverseKinematicsUR5, transformRobotParameter
from copy import deepcopy
from std_msgs.msg import Int32
# import cv2

m_cali = np.identity(4)
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
SPEED = 3
# pro_pub = rospy.Publisher('/netsend', Int32MultiArray, queue_size=1)

# # , [0.340,-0.261], [449, 103]
# def netsend(msg, flag=-1, need_unpack=True):
#     global pro_pub, gesture_id
#     if msg:
#         if flag != 1:
#             rospy.loginfo("flag is {}. msg is {}".format(flag, msg))
#         if need_unpack:
#             send = []
#             for i in range(len(msg)):
#                 send.append(int(msg[i][0]))
#                 send.append(int(msg[i][1]))
#             a = deepcopy(send)
#             a.append(flag)
#         else:
#             a = deepcopy(msg)
#             a.append(flag)
#         pro_pub.publish(Int32MultiArray(data=a))
    
# def coord_converter(x, y):
#     pts2 = np.array([[(-0.343, -0.242), (-0.342, -0.518), (-0.339, -0.751), (0.040, -0.233), (0.015, -0.501), (0.061, -0.768), (0.437, -0.235), (0.451, -0.522), (0.448, -0.755)]])
#     pts1 = np.array([[96, 28], [95, 226], [98, 388], [318, 24], [313, 213], [325, 393], [536, 27],[537, 228],[533, 390]])
#     M, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC,5.0)
#     solusion = np.matmul(M,np.array([x, y, 1]))
#     solusion = solusion/solusion[2]
#     return solusion[0], solusion[1]

def transform_inverse(mat):
        rot = mat[:3, :3]
        trans = mat[:3, -1]
        inv = np.identity(4)
        inv[:3, :3] = rot.T
        inv[:3, -1] = -np.matmul(rot.T, trans.reshape(-1, 1)).ravel()
        return inv

def compute_transform(m_cali, m_tag, m):
    m_tag_inv = transform_inverse(m_tag)
    temp = np.matmul(m_tag_inv, m_cali)
    return np.matmul(m, temp)

def output_grasp(mat):
        quat = tf.transformations.quaternion_from_euler(3.14, 0, -3.14)
        dest_m = self.Transformer.fromTranslationRotation([0, 0, 0], quat)

        return np.matmul(dest_m, mat)

#tool0 is ee
class pick_place:
    def __init__(self):
        #/vel_based_pos_traj_controller/
        self.client = actionlib.SimpleActionClient('icl_phri_ur5/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = JOINT_NAMES
        print ("Waiting for server...")
        self.client.wait_for_server()
        print ("Connected to server")
        joint_states = rospy.wait_for_message("joint_states", JointState)
        print(joint_states)
        joint_states = list(deepcopy(joint_states).position)
        del joint_states[-1]
        self.joints_pos_start = np.array(joint_states)
        print ("Init done")
        self.listener = tf.TransformListener()
        self.Transformer = tf.TransformerROS()

        joint_weights = [12,5,4,3,2,1]
        self.ik = InverseKinematicsUR5()
        self.ik.setJointWeights(joint_weights)
        self.ik.setJointLimits(-pi, pi)

        # self.sub = rospy.Subscriber('/target_position', Int32MultiArray, self.pickplace_cb)
        # pickplace_sub = message_filters.Subscriber('pick', Int32)
        # tag_sub = message_filters.Subscriber('tag', Int32)

        # ts = message_filters.ApproximateTimeSynchronizer([pickplace_sub, tag_sub], 10, 0.1, allow_headerless=True)
        # ts.registerCallback(self.pickplace_cb)

        self.sub_pickplace = rospy.Subscriber('/CHOICE', String, self.pickplace_cb)

        # self.gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        # self.gripper_ac.wait_for_server()
        # self.gripper_ac.initiate()
        # self.gripper_ac.send_goal(0.10)
        # self.gripper_ac.wait_for_result()
        # self.cancel = False

    def define_grasp(self, position):
        quat = tf.transformations.quaternion_from_euler(3.14, 0, -3.14)
        dest_m = self.Transformer.fromTranslationRotation(position, quat) 
        return dest_m
    
    def move(self, dest_m):
        #current_m = transformRobotParameter(self.joints_pos_start)
        qsol = self.ik.findClosestIK(dest_m,self.joints_pos_start)
        
        if qsol is not None and not self.cancel:
            if qsol[0] < 0:
                qsol[0] += pi
            else:
                qsol[0] -= pi
            self.goal.trajectory.points = [
                JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=qsol.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(SPEED)),
            ]

            try:
                self.client.send_goal(self.goal)
                self.joints_pos_start = qsol
                self.client.wait_for_result()
            except:
                raise
        elif qsol is None:
            rospy.loginfo("fail to find IK solution")
        elif self.cancel:
            rospy.logwarn("this goal canceled")
    
    def single_exuete(self, msg):

        pre_position = output_grasp(mat.copy())
        pre_position[2, -1] += 0.1
        post_position = output_grasp(mat.copy())
        post_position[2, -1] += 0.1
        grasp_position = output_grasp(mat.copy())
        rospy.loginfo("let's go to the pre location")
        self.move(pre_position)
        #rospy.sleep(1)
        rospy.loginfo("let's do this")
        self.move(grasp_position)
        #rospy.sleep(1)
        if mode == "pick":
            self.gripper_ac.send_goal(0)
        elif mode == "place":
            self.gripper_ac.send_goal(0.10)
        self.gripper_ac.wait_for_result()
        rospy.loginfo("move out")
        self.move(post_position)
        #rospy.sleep(1)

    def pair_exuete(self, pick, place):
        rospy.loginfo("here we go pair")
        if pickn and place:
            self.single_exuete(pick, "pick")
            self.single_exuete(place, "place")

    
    def pickplace_cb(self, m, m_tag):
 
        self.cancel = False
        pick = compute_transform(m_cali, m_tag, m_pick)
        place = compute_transform(m_cali, m_tag, m_place)

        self.pair_exuete(pick, place)
        if self.cancel:
            self.cancel = False

            joint_states = rospy.wait_for_message("joint_states", JointState)
            joint_states = list(deepcopy(joint_states).position)
            del joint_states[-1]
            self.joints_pos_start = np.array(joint_states)

            self.client.wait_for_result()
        rospy.loginfo("let's go and get some rest")
        rest_position = self.define_grasp([0.405, 0.010, 0.342])
        self.move(rest_position)





if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    task = pick_place()
    rospy.spin()
