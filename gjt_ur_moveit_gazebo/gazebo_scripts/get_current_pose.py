#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import tf.transformations as tft
import sys
rospy.init_node('get_end_effector_rpy', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group_name = "arm" 
group = moveit_commander.MoveGroupCommander(group_name)
current_pose = group.get_current_pose().pose
print("current_pose:",current_pose)
quaternion = (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
rpy = tft.euler_from_quaternion(quaternion)

