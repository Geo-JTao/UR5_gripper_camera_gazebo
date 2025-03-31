#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from std_srvs.srv import Empty
from math import pi 
import sys, tf, time, os,math
from copy import deepcopy
import random,os
RED = '\033[91m'
PINK = '\033[95m'
BLUE = '\033[94m'
ENDC = '\033[0m'
YELLOW  = '\033[33m'
GREEN_BOLD = '\033[92;1m'
GREEN = '\033[92m'
class Spawn_object():
    def __init__(self,is_testing,workspace_limits,obj_number) :
        self.USE_OBJECTS = [
            'chewinggum','eraser', 'salt', 
            'cappuccino', 'cleaner', 'cube', 'glue',
             'shampoo', 'sticky_notes', 'sweetener',
            'banana'
        ]
        self.INITIAL_POS = []
        self.INITIAL_TEST_POS=[]
        for j in range(obj_number,obj_number+1):
            for i in range(1, 11):
                self.INITIAL_POS.append('{}_'.format(j) + str(i))
                self.INITIAL_TEST_POS.append('{}_'.format(j) + str(i))
        # self.INITIAL_TEST_POS = ['3_1', '3_2', '3_3', '4_1', '4_2', '4_3', '4_4', '5_1', '5_2', '5_3', 
        #                                                                                             '6_1', '6_2', '6_3', '6_4', '7_1', '7_2', '8_1']
        self.spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel,persistent=True)
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel,persistent=True)
        self.set_object_position_service = rospy.ServiceProxy('gazebo/set_model_state', SetModelState,persistent=True)
        self.get_object_position_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState,persistent=True)
        self.get_object_name_service =  rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.reset_sim_service =  rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.is_testing = is_testing
        self.workspace_limits = workspace_limits
        self.model_on_desk = []
        self.model_on_desk_pos = []
        self.model_on_desk_distances=[]
        self.reset = False
        self.object_number = obj_number
        self.initial_scene_index = 0
        self.delete_all_objects()
        self.input_all_object()

    def input_all_object(self):
        print('\033[32m' + '---input_all_object----' + '\033[0m')
        self.object_spawn()
        self.random_choice_env(self.initial_scene_index)
        self.initial_scene_index +=1
        if not self.is_testing:
            if self.initial_scene_index == len(self.INITIAL_POS):
                self.initial_scene_index =0
        else:
            if self.initial_scene_index == len(self.INITIAL_TEST_POS):
                self.initial_scene_index =0
        self.get_distance_after_push()

    def object_spawn(self):
        rospy.wait_for_service("gazebo/spawn_sdf_model",timeout=5)
        goal = np.array([-0.250 , -0.250,  0 ,0, 0, 0])
        for i in self.USE_OBJECTS:
            rand_num = 1
            goal[0] = goal[0] - 0.05*rand_num
            goal[1] = goal[1] - 0.05*rand_num
            rand_num +=1
            orient = Quaternion(*tf.transformations.quaternion_from_euler(goal[3],goal[4],goal[5]))
            origin_pose = Pose(Point(goal[0],goal[1],goal[2]), orient)
            filename = os.path.expandvars('$HOME/.gazebo/models/%s/model.sdf' % i)
           
            with open(filename,"r") as f:
                reel_xml = f.read()
            pose = deepcopy(origin_pose)
            print('The {} model is being generated'.format(i))
            self.spawn(i, reel_xml, "", pose, "world")
            rospy.sleep(0.5)
        rospy.sleep(1)

    def delete_all_objects(self):
        resp = self.get_object_name_service()
        if resp.success:
            model_names = resp.model_names
            models_to_keep = ['ground_plane', 'base_table', 'ur5_gripper','red_box','pink_box']
            for model_name in model_names:
                if model_name not in models_to_keep:
                    self.delete_model(model_name)
                    print('The {} model was deleted'.format(model_name))
                    rospy.sleep(0.5)
        self.model_on_desk = []
        self.model_on_desk_pos = []
        self.model_on_desk_distances=[]
        rospy.sleep(0.2)

    def reset_obj_pos(self):
        print('Resetting the object pose ...')
        resp = self.get_object_name_service()
        if resp.success:
            model_names = resp.model_names
            models_to_keep = ['ground_plane', 'base_table', 'ur5_gripper']
            goal = np.array([-0.250 , -0.250,  0 ,0, 0, 0])
            rand_num = 1
            for i in model_names:
                if i not in models_to_keep:
                    goal[0] = goal[0] - 0.015*rand_num
                    goal[1] = goal[1] - 0.015*rand_num
                    rand_num +=1
                    orient = Quaternion(*tf.transformations.quaternion_from_euler(goal[3],goal[4],goal[5]))
                    objstate = SetModelStateRequest()
                    objstate.model_state.model_name = i
                    objstate.model_state.pose.position.x = goal[0]
                    objstate.model_state.pose.position.y = goal[1]
                    objstate.model_state.pose.position.z = goal[2]
                    objstate.model_state.pose.orientation = orient
                    objstate.model_state.reference_frame = "world"
                    self.set_object_position_service(objstate)
                    rospy.sleep(0.5)
        self.model_on_desk = []
        self.model_on_desk_pos = []
        self.model_on_desk_distances=[]
        rospy.sleep(0.2)

    # model_name x,y,z,r,p,y
    def random_choice_env(self,initial_index):
        if not self.is_testing:
            model_pos_txt = self.INITIAL_POS
            num_ind = model_pos_txt[initial_index]
            a = num_ind.split('_') # 3_1.txt ---> a[0]=3,a[1]=1
            model_env = 'train'
            base_path = 'src/gjt_ur_moveit_gazebo/env_info/train_data_with_banana/object_{}'.format(a[0])
            target_path = os.path.join(base_path, a[0]) + '/%s.txt'%(num_ind)
        else :
            model_pos_txt = self.INITIAL_TEST_POS
            num_ind = model_pos_txt[initial_index]
            a = num_ind.split('_')
            model_env = 'test'
            base_path = 'src/gjt_ur_moveit_gazebo/env_info/test_data_with_banana/object_{}'.format(a[0])
            target_path = os.path.join(base_path, a[0]) + '/%s.txt'%(num_ind)
        print('\033[33m{} ---> Current_env: {}.txt\033[0m'.format(model_env, num_ind))
        with open(target_path, 'r') as fd:
            obj_pos = fd.readlines()
        target_obj = []
        for i in obj_pos:
            i = i.rstrip()
            print(i)
            name, pos = i.split()
            if not i:  
                continue
            pos = pos.split(',')
            target_obj.append([name, [float(j) for j in pos]])
        self.move2desk(target_obj)

    def move2desk(self, name_pos_list):
        if self.is_testing:
            print('\033[32m' + 'test------move2desk------' + '\033[0m')
            rand_x, rand_y,rand_R,rand_P, rand_Y = 0,0,0,0,0
        else :
            print('\033[32m' + 'train------move2desk_random------' + '\033[0m')
            rand_x, rand_y = np.random.uniform(-0.01,0.01), np.random.uniform(-0.01,0.01)
            rand_R = np.random.uniform(-math.pi,math.pi)
            rand_P, rand_Y = np.random.uniform(-math.pi,math.pi),np.random.uniform(-math.pi,math.pi)
        for obj in name_pos_list:
            orient = Quaternion(*tf.transformations.quaternion_from_euler(obj[1][3], obj[1][4], obj[1][5])) 
            objstate = SetModelStateRequest()
            objstate.model_state.model_name = obj[0]
            self.model_on_desk.append(objstate.model_state.model_name)
            objstate.model_state.pose.position.x = obj[1][0]
            objstate.model_state.pose.position.y = obj[1][1]
            objstate.model_state.pose.position.z = obj[1][2]
            self.model_on_desk_pos.append([obj[1][0],obj[1][1]])
            objstate.model_state.pose.orientation = orient
            objstate.model_state.reference_frame = "world"
            self.set_object_position_service(objstate)
            time.sleep(0.5)
        with open('src/gjt_ur_moveit_gazebo/env_info/obj_on_desk.txt', 'w') as f:
            for model_name in self.model_on_desk:
                f.write(model_name + ',')
        for i in range(len(self.model_on_desk_pos)):
            for j in range(i+1, len(self.model_on_desk_pos)):
                dist = np.sqrt((self.model_on_desk_pos[i][0] - self.model_on_desk_pos[j][0])**2 + 
                            (self.model_on_desk_pos[i][1] - self.model_on_desk_pos[j][1])**2)
                self.model_on_desk_distances.append(dist)

    def reset_env_from_txt(self):
        print('\033[33m' + '---Reset environment----' + '\033[0m')
        self.reset_obj_pos()
        self.random_choice_env(self.initial_scene_index)
        self.initial_scene_index +=1
        if not self.is_testing:
            if self.initial_scene_index == len(self.INITIAL_POS):
                self.initial_scene_index =0
        else:
            if self.initial_scene_index == len(self.INITIAL_TEST_POS):
                self.initial_scene_index =0
        self.get_distance_after_push()
        self.reset = True

    def if_obj_in_env(self):
        x_min, x_max, y_min, y_max, z_min, z_max = self.workspace_limits
        Get_Pos = GetModelStateRequest()
        for i in self.model_on_desk:
            Get_Pos.model_name = i
            obj_x = self.get_object_position_service(Get_Pos).pose.position.x
            obj_y = self.get_object_position_service(Get_Pos).pose.position.y
            if obj_x < x_min or obj_x >  x_max  or obj_y > y_max or obj_y < y_min:
                return False
        return True
   
    def get_distance_after_push(self):
        self.model_on_desk_pos=[]
        self.model_on_desk_distances=[]
        Get_Pos = GetModelStateRequest()
        for i in self.model_on_desk:
            Get_Pos.model_name = i
            obj_x = self.get_object_position_service(Get_Pos).pose.position.x
            obj_y = self.get_object_position_service(Get_Pos).pose.position.y
            self.model_on_desk_pos.append([obj_x,obj_y])
        for i in range(len(self.model_on_desk_pos)):
            for j in range(i+1, len(self.model_on_desk_pos)):
                dist = np.sqrt((self.model_on_desk_pos[i][0] - self.model_on_desk_pos[j][0])**2 + 
                            (self.model_on_desk_pos[i][1] - self.model_on_desk_pos[j][1])**2)
                self.model_on_desk_distances.append(dist)

    def get_grasp_model_name_pos(self):
        # get objects' z-axis height
        grasp_model_z = 0
        grasp_model_name = None
        grasp_model_x = None
        grasp_model_y = None
        Get_Pos = GetModelStateRequest()
        for i in self.model_on_desk:
            Get_Pos.model_name = i
            obj_z = self.get_object_position_service(Get_Pos).pose.position.z
            if obj_z > grasp_model_z:
                grasp_model_z = obj_z
                grasp_model_name = i
                grasp_model_x = self.get_object_position_service(Get_Pos).pose.position.x
                grasp_model_x = self.get_object_position_service(Get_Pos).pose.position.y
        return [grasp_model_name, grasp_model_z, grasp_model_x, grasp_model_y]

    def remove_grasp_obj(self,obj_name):
        orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0)) 
        objstate = SetModelStateRequest()
        objstate.model_state.model_name = obj_name
        objstate.model_state.pose.position.x = -0.3 
        objstate.model_state.pose.position.y = -0.3
        objstate.model_state.pose.position.z = 0
        objstate.model_state.pose.orientation = orient
        objstate.model_state.reference_frame = "world"
        self.set_object_position_service(objstate)
        self.model_on_desk.remove(obj_name)

if __name__ == '__main__':

    x_min = 0.4-0.1
    x_max = 0.975-0.1
    y_min = -0.2
    y_max = 0.24
    workspace_limits = (x_min,x_max,y_min,y_max, 0.08, 0.15)
    spawn =  Spawn_object(is_testing=False,workspace_limits=workspace_limits,obj_number=9)
    while True:
        user_input =  raw_input("Press 'c' to reset environment, or 'q' to quit: ")
        if user_input == 'c':
            spawn.reset_env_from_txt()
            if_obj_in_env = spawn.if_obj_in_env()
            print('if_obj_in_env:{}'.format(if_obj_in_env))
            print('distances_lenth:{}'.format(len(spawn.model_on_desk_distances)))

        elif user_input == 'q':
            break


