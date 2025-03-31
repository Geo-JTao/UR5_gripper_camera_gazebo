#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from move_gripper import control_gripper
from move_robot import MoveIt_Control
from saveimg import ImageSaver
from gjt_ur_moveit_gazebo.srv import grasp_pose,grasp_poseResponse
import numpy as np
from math import pi
import math
import tf.transformations as tft

class UR_Grasp:
    def __init__(self):
        self.move = MoveIt_Control()
        self.camera = ImageSaver()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        control_gripper(0)

    def get_pos_in_camera(self,pix_x,pix_y,cam_intrinsics):
        u = int(float(pix_x)/224*640)
        v = int(float(pix_y)/224*480)
        camera_points=[]
        ppx_up =  cam_intrinsics[0][2]
        ppy_up =  cam_intrinsics[1][2]
        pos_z = 0.769
        pos_x = np.multiply(u  - ppx_up,pos_z / cam_intrinsics[0][0])
        pos_y = np.multiply(v  - ppy_up,pos_z / cam_intrinsics[1][1])
        camera_point = np.asarray([pos_x,pos_y,pos_z])
        # print("camera_point : {}".format(camera_point))
        camera_points.append(camera_point)
        return camera_points

    def get_base_link_pos(self,x,y,z):
        cam_pos = np.array([x,y,z,1])
        tran_R_1 = np.dot(self.move.cam_pose[0:3,0:3],cam_pos[0:3])
        tran_T_2 = tran_R_1 + self.move.cam_pose[0:3,3]
        base_link_pos = tran_T_2
        # print('base_link_pos : {}'.format(base_link_pos))
        return base_link_pos

    def world_to_base_link_pos(self,x,y,z):
        world_position = [x,y,z]
        point_source = PointStamped()
        point_source.header.frame_id = "world"
        point_source.header.stamp = rospy.Time.now()
        point_source.point.x = world_position[0]
        point_source.point.y = world_position[1]
        point_source.point.z = world_position[2]
        point_target = self. buffer.transform(point_source,"base_link",timeout=rospy.Duration(0.1))
        base_link_pos = [point_target.point.x,point_target.point.y,point_target.point.z]
        print("base_link : {}".format(base_link_pos))
        return base_link_pos

    def grasp(self,base_link_pos,angle,gripper_value):
        base_link_pos[2] = 0.0139576627708
        pre_grasp_pos =  [base_link_pos[0],base_link_pos[1],base_link_pos[2]+0.165+0.03]
        grasp_pos =  [base_link_pos[0],base_link_pos[1],base_link_pos[2]+0.165-0.025]
        try:
            control_gripper(0)
            self.move.move_p(pre_grasp_pos,a=0.6,v=0.6)
            joint_value = self.move.arm.get_current_joint_values()
            if (abs(angle)>90):
                angle = -(180-abs(angle))
            angle =abs(90-abs(angle))
            tool_rotation_angle = (angle /360 *2* np.pi) 
            joint_value[5] = joint_value[5] + tool_rotation_angle
            self.move.move_j(joint_value)
            rospy.sleep(0.2)
            current_pose = self.move.arm.get_current_pose().pose
            quaternion = (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
            rpy = tft.euler_from_quaternion(quaternion)
            self.move.move_p(grasp_pos,rpy,a=0.5,v=0.5)
            rospy.sleep(0.5)
            control_gripper(gripper_value)
            rospy.sleep(0.5)
            self.move.move_p(pre_grasp_pos,a=0.5,v=0.5)
            self.move.go_home()
            rospy.sleep(4)
            control_gripper(0)
            rospy.sleep(1)
            grasp_success = True
        except:
            grasp_success = False
        return grasp_success


    def push(self,base_link_pos,angle):
        control_gripper(0.78)
        base_link_pos[2] = 0.0139576627708 
        pre_push_pos =  [base_link_pos[0],base_link_pos[1],base_link_pos[2]+0.165+0.03]
        push_pos =  [base_link_pos[0],base_link_pos[1],base_link_pos[2]+0.165-0.025]
        push_length = 0.13
        angle_rad = math.radians(angle)
        target_x = base_link_pos[0]  + push_length * math.cos(angle_rad)
        target_y = base_link_pos[1]  + push_length * math.sin(angle_rad)
        push_target_pos = [target_x,target_y,push_pos[2]]
        try:
            rospy.sleep(0.1)
            self.move.move_p(pre_push_pos,a=0.7,v=0.7)
            rospy.sleep(0.1)
            self.move.move_p(push_pos,a=0.7,v=0.7)
            joint_value = self.move.arm.get_current_joint_values()
            if (abs(angle)>90):
                angle = -(180-abs(angle))
            angle =abs(90-abs(angle))
            tool_rotation_angle = (angle /360 *2* np.pi) 
            joint_value[5] = joint_value[5] +tool_rotation_angle
            self.move.move_j(joint_value)
            rospy.sleep(0.2)
            current_pose = self.move.arm.get_current_pose().pose
            quaternion = (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
            rpy = tft.euler_from_quaternion(quaternion)
            self.move.move_p(push_target_pos,rpy,a=0.5,v=0.5)
            self.move.go_home()
            control_gripper(0)
            push_success = True
        except:
            push_success = False
        return push_success

    def open_gripper_to_reset(self):
        control_gripper(0)
        rospy.sleep(1) 

def grasp_callback(req):
    rospy.loginfo("Responding to a request...")
    cam_position = [req.grasppose_x,req.grasppose_y,req.grasppose_z]
    print("Received data:{}".format(cam_position))
    baselink_pos = ur5_grasp.get_base_link_pos(cam_position[0],cam_position[1],cam_position[2])
    angle = req.grasppose_R
    action_id = req.grasppose_P
    if action_id == 0:
        ur5_grasp.push(baselink_pos,angle)
    elif action_id == 1:
        ur5_grasp.grasp(baselink_pos,angle,0.5)
    elif action_id == 2:
        ur5_grasp.open_gripper_to_reset()

    response = grasp_poseResponse()
    response.success = True
    return response

if __name__ == "__main__":
    ur5_grasp = UR_Grasp()
    ur5_grasp.camera.save_images(color_filename="saved_picture/color{}.png",
                            depth_filename="saved_picture/depth{}.png".format(ur5_grasp.camera.counter))
    print("Saving {} image".format(ur5_grasp.camera.counter))
    server = rospy.Service("moveit_grasp",grasp_pose,grasp_callback )
    print("***************************")
    print("Waiting for client request......")
    print("***************************")
    rospy.spin()

    
 
       

   