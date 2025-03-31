#! /usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import threading
import rospy
from gjt_ur_moveit_gazebo.srv import grasp_pose, grasp_poseRequest

def get_instance_angle(mask):
    binary_mask = np.zeros(mask.shape, dtype=np.uint8)  
    binary_mask[mask == 255] = 255
    cv2.imshow('binary_mask', binary_mask)
    cv2.waitKey(0)
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) < 1:
        return None
    points = contours[0]
    [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
    angle = np.arctan2(vy, vx) * 180 / np.pi
    start_point = (int(x - vx * 100), int(y - vy * 100))
    end_point = (int(x + vx * 100), int(y + vy * 100))
    return angle, start_point,end_point

def show_angle(suction_pts_image,start_points, end_points, contour_angles):
    canvas = np.zeros_like(suction_pts_image)
    for start_point, end_point, angle in zip(start_points, end_points, contour_angles):
        line_length = math.sqrt((end_point[0] - start_point[0]) ** 2 + (end_point[1] - start_point[1]) ** 2)
        shortened_length = line_length / 1.5
        angle_rad = math.atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
        new_start_x = int(start_point[0] + shortened_length * math.cos(angle_rad))
        new_start_y = int(start_point[1] + shortened_length * math.sin(angle_rad))
        new_start_point = (new_start_x, new_start_y)
        new_end_x = int(end_point[0] - shortened_length * math.cos(angle_rad))
        new_end_y = int(end_point[1] - shortened_length * math.sin(angle_rad))
        new_end_point = (new_end_x, new_end_y)
        cv2.arrowedLine(canvas, new_end_point,new_start_point,  (0, 255, 255), 2)
        line_center = ((new_start_point[0] + new_end_point[0]) // 2, (new_start_point[1] + new_end_point[1]) // 2)
        line_length = 30 
        line_end = (line_center[0] + line_length, line_center[1])
        cv2.line(canvas, line_center, line_end, (0 ,0, 255), 2)
        text_position = (int((start_point[0] + end_point[0]) / 2), int((start_point[1] + end_point[1]) / 2))
        angle=np.round(angle, 1)
        cv2.putText(canvas, "{}".format(angle), text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
    suction_pts_image = cv2.addWeighted(suction_pts_image, 1, canvas, 1, 0)
    return suction_pts_image

def show_img(name,image):
    cv2.imshow(name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

class MyThread(threading.Thread):
    def __init__(self, prompt):
        threading.Thread.__init__(self)
        self.prompt = prompt
        self.text = None
    
    def run(self):
        self.text = input(self.prompt)

def get_real_angle(angle):
    if (abs(angle)>90):
        angle = -(180-abs(angle))
    angle =abs(90-abs(angle))
    return angle

def make_object_info_dict_oneimg(camera_points, contour_angles):
    camera_points_nums = [float(num) for num in camera_points[0]]
    contour_angles_nums = [float(num) for num in contour_angles[0]]
    object_info_dict = {
        'camera_point': camera_points_nums,
        'contour_angle': contour_angles_nums,
    }
    return object_info_dict

def send_pose_to_robot_oneimg(object_info_dict):
    move_client = rospy.ServiceProxy("moveit_grasp", grasp_pose)
    rospy.wait_for_service("moveit_grasp")
    start_camera_point = object_info_dict['camera_point']
    start_contour_angle =  object_info_dict['contour_angle']
    start_x, start_y, start_z = start_camera_point
    start_angle = get_real_angle(start_contour_angle[0])
    print("Grasp: X: {:.2f}, Y: {:.2f}, Z: {:.2f}, angle: {}".format(start_x,start_y,start_z,start_angle))
    move_req = grasp_poseRequest()
    pos_x,pos_y,pos_z=start_x,start_y,start_z
    angle_x ,angle_y,angle_z = -1.4788849627179739, -0.014592622655851381, 0.08230054773210334
    move_req.grasppose_x, move_req.grasppose_y, move_req.grasppose_z = pos_x,pos_y,pos_z
    move_req.grasppose_R, move_req.grasppose_P, move_req.grasppose_Y =angle_x ,angle_y,angle_z
    result = move_client.call(move_req)
    return  result
