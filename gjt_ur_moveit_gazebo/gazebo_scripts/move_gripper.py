#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def control_gripper(value):
    # rospy.init_node('gripper_control_node', anonymous=True)
    client = actionlib.SimpleActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    trajectory = JointTrajectory()
    trajectory.joint_names = ['gripper_finger1_joint']  

    point = JointTrajectoryPoint()
    point.positions = [value] 
    point.time_from_start = rospy.Duration(1.0)  

    trajectory.points.append(point)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory

    client.send_goal(goal)
    client.wait_for_result()

def open_gripper():
    print("Open the gripper...")
    control_gripper(0)

def close_gripper():
    print("close the gripper...")
    control_gripper(1)

if __name__ == '__main__':
    open_gripper()
    rospy.sleep(1)
    close_gripper()

