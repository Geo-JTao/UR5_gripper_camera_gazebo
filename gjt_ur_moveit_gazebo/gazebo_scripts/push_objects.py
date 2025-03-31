#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import struct
import math
import numpy as np
import cv2
import torch
from scipy import ndimage
from moveitServer import UR_Grasp
from spawn_model import Spawn_object
from collections import OrderedDict
import numpy as np
import torch  
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision
import matplotlib.pyplot as plt
import rospy
import argparse, os,random
import warnings
warnings.filterwarnings("ignore", message="nn.init.kaiming_normal is now deprecated in favor of nn.init.kaiming_normal_.")
warnings.filterwarnings("ignore", message="size_average and reduce args will be deprecated, please use reduction='none' instead.")
warnings.filterwarnings("ignore", message="nn.Upsample is deprecated. Use nn.functional.interpolate instead.")
warnings.filterwarnings("ignore", message="Default upsampling behavior when mode=bilinear is changed to align_corners=False since 0.4.0*")

RED = '\033[91m'
PINK = '\033[95m'
BLUE = '\033[94m'
ENDC = '\033[0m'
YELLOW  = '\033[33m'
YELLOW_BOLD   = '\033[33;1m'
GREEN_BOLD = '\033[92;1m'
GREEN = '\033[92m'
def show_push_pred(rgb_img_show,pix_x, pix_y,angle):
    radius = 5
    color = (0, 0, 255)  
    thickness = 2
    cv2.circle(rgb_img_show, ( pix_x, pix_y ), radius, color, thickness)
    length = 35
    end_x = int(pix_x + length * math.cos(math.radians(-angle)))
    end_y = int(pix_y + length * math.sin(math.radians(-angle)))
    color = (0, 255, 0) 
    thickness = 2
    cv2.arrowedLine(rgb_img_show, ( pix_x, pix_y ), (end_x, end_y), color, thickness)
    cv2.imshow('show_push_pred', rgb_img_show)
    cv2.waitKey(100)
    # cv2.destroyAllWindows()
def get_outline_from_depth(depth_img):
    mask = np.zeros(depth_img.shape, dtype=np.uint8)   
    mask[depth_img == 255] = 255
    push_binary_segment = mask.copy()
    push_kernel = np.ones((7, 7), np.uint8) 
    push_delete_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    push_mor_segment = cv2.morphologyEx(push_binary_segment, cv2.MORPH_OPEN, push_delete_small)
    push_dilate_kernel = np.ones((15, 15), np.uint8)
    push_dilate_1 = cv2.dilate(push_mor_segment, push_kernel)
    push_dilate_2 = cv2.dilate(push_mor_segment, push_dilate_kernel)
    outline = push_dilate_2 - push_dilate_1
    return mask,outline
def get_model_input(color_heightmap, depth_heightmap, is_volatile=True, specific_rotation=-1):
    color_heightmap_2x = ndimage.zoom(color_heightmap, zoom=[2,2,1], order=0)
    depth_heightmap_2x = ndimage.zoom(depth_heightmap, zoom=[2,2], order=0)
    assert(color_heightmap_2x.shape[0:2] == depth_heightmap_2x.shape[0:2])
    diag_length = float(color_heightmap_2x.shape[0]) * np.sqrt(2)  
    diag_length = np.ceil(diag_length/32)*32 
    padding_width = int((diag_length - color_heightmap_2x.shape[0])/2)
    color_heightmap_2x_r = np.pad(color_heightmap_2x[:,:,0], padding_width, 'constant', constant_values=0)
    color_heightmap_2x_r.shape = (color_heightmap_2x_r.shape[0], color_heightmap_2x_r.shape[1], 1)
    color_heightmap_2x_g = np.pad(color_heightmap_2x[:,:,1], padding_width, 'constant', constant_values=0)
    color_heightmap_2x_g.shape = (color_heightmap_2x_g.shape[0], color_heightmap_2x_g.shape[1], 1)
    color_heightmap_2x_b = np.pad(color_heightmap_2x[:,:,2], padding_width, 'constant', constant_values=0)
    color_heightmap_2x_b.shape = (color_heightmap_2x_b.shape[0], color_heightmap_2x_b.shape[1], 1)
    color_heightmap_2x = np.concatenate((color_heightmap_2x_r, color_heightmap_2x_g, color_heightmap_2x_b), axis=2)
    depth_heightmap_2x = np.pad(depth_heightmap_2x, padding_width, 'constant', constant_values=0)
    image_mean = [0.34751591, 0.29237225, 0.17272882]
    image_std = [0.07591467, 0.08059952, 0.09573705]
    input_color_image = color_heightmap_2x.astype(float)/255
    for c in range(3):
        input_color_image[:,:,c] = (input_color_image[:,:,c] - image_mean[c])/image_std[c]

    image_mean = [0.9791, 0.9791,0.9791]
    image_std = [0.1227, 0.1227, 0.1227]
    cp_depth_heightmap = depth_heightmap_2x.copy()
    cp_depth_heightmap.shape = (cp_depth_heightmap.shape[0], cp_depth_heightmap.shape[1], 1)
    input_depth_image = np.concatenate((cp_depth_heightmap, cp_depth_heightmap, cp_depth_heightmap), axis=2)
    for c in range(3):
        input_depth_image[:,:,c] = (input_depth_image[:,:,c] - image_mean[c])/image_std[c]

    input_color_image.shape = (input_color_image.shape[0], input_color_image.shape[1], input_color_image.shape[2], 1)
    input_depth_image.shape = (input_depth_image.shape[0], input_depth_image.shape[1], input_depth_image.shape[2], 1)
    input_color_data = torch.from_numpy(input_color_image.astype(np.float32)).permute(3,2,0,1)
    input_depth_data = torch.from_numpy(input_depth_image.astype(np.float32)).permute(3,2,0,1)
    return input_color_data,input_depth_data
    
def policynms_push( push_predictions, num_rotations, valid_depth_heightmap):
    push_action_nms_value = 0
    push_index = 0
    push_final_pixel = None
    for num, push_prediction in enumerate(push_predictions):
        push_mask = np.zeros((224, 224), np.uint8)
        push_pixel = np.unravel_index(np.argmax(push_prediction), push_prediction.shape)
        push_predicted_value = np.max(push_prediction)
        push_roi = [[min(max(push_pixel[1] - 4, 0), 223), min(max(push_pixel[0] + 11, 0), 223)],
                [min(max(push_pixel[1] + 4, 0), 223), min(max(push_pixel[0] + 20, 0), 223)]]
        push_mask = cv2.rectangle(push_mask, (push_roi[0][0], push_roi[0][1]), (push_roi[1][0], push_roi[1][1]), (255, 255, 255), -1)
        push_rotate = cv2.getRotationMatrix2D((push_pixel[1], push_pixel[0]), num*360/num_rotations, 1)
        push_final = cv2.warpAffine(push_mask, push_rotate, (224, 224))
        push_ROI_multiply = np.zeros(push_final.shape, dtype=np.uint8)
        push_ROI_count = np.zeros(push_final.shape, dtype=np.uint8)
        push_ROI_multiply[push_final >= 100] = 255
        push_ROI_count[push_final >= 100] = 1
        push_total_pixel = np.sum(push_ROI_count)
        push_depth_nms = np.multiply(valid_depth_heightmap, push_ROI_count)
        push_nms_count = np.zeros(push_depth_nms.shape, dtype=np.uint8)
        push_nms_count[push_depth_nms >= 250] = 1
        push_obj_pixel = np.sum(push_nms_count)
        push_prob = float(push_obj_pixel) / (float(push_total_pixel) + 1)
        push_temp = push_prob * push_predicted_value
        if push_temp >= push_action_nms_value:
            push_action_nms_value = push_temp
            push_index = num
            push_final_pixel = push_pixel
    if push_action_nms_value == 0:
        push_locate_part = np.unravel_index(np.argmax(push_predictions), push_predictions.shape)
        push_predicted_value = np.max(push_predictions)
    else:
        push_locate_part = tuple([push_index, push_final_pixel[0], push_final_pixel[1]])
        push_predicted_value = push_action_nms_value

    return push_locate_part, push_predicted_value

class push_net(nn.Module):
    def __init__(self, use_cuda = True):
        super(push_net, self).__init__()
        self.use_cuda = use_cuda

        self.push_color_trunk = torchvision.models.densenet.densenet121(pretrained=True)
        self.push_depth_trunk = torchvision.models.densenet.densenet121(pretrained=True)
        self.num_rotations = 16
        self.pushnet = nn.Sequential(OrderedDict([
            ('push-norm0', nn.BatchNorm2d(2048)),
            ('push-relu0', nn.ReLU(inplace=True)),
            ('push-conv0', nn.Conv2d(2048, 64, kernel_size=1, stride=1, bias=False)),
            ('push-norm1', nn.BatchNorm2d(64)),
            ('push-relu1', nn.ReLU(inplace=True)),
            ('push-conv1', nn.Conv2d(64, 1, kernel_size=1, stride=1, bias=False))
        ]))

        for m in self.named_modules():
            if 'push-' in m[0]:
                if isinstance(m[1], nn.Conv2d):
                    nn.init.kaiming_normal_(m[1].weight.data)
                elif isinstance(m[1], nn.BatchNorm2d):
                    m[1].weight.data.fill_(1)
                    m[1].bias.data.zero_()
        self.push_interm_feat = []
        self.push_output_prob = []

    def forward(self, input_color_data, input_depth_data, is_volatile=True, specific_rotation=-1):
        if is_volatile:
            with torch.no_grad():
                push_interm_feat = []
                push_output_prob = []
                for rotate_idx in range(self.num_rotations):
                    rotate_theta = np.radians(rotate_idx*(360/self.num_rotations))
                    affine_mat_before = np.asarray([[np.cos(-rotate_theta), np.sin(-rotate_theta), 0],[-np.sin(-rotate_theta), np.cos(-rotate_theta), 0]])
                    affine_mat_before.shape = (2,3,1)
                    affine_mat_before = torch.from_numpy(affine_mat_before).permute(2,0,1).float()
                    if self.use_cuda:
                        flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False).cuda(), input_color_data.size())
                    else:
                        flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False), input_color_data.size())
                    if self.use_cuda:
                        rotate_color = F.grid_sample(Variable(input_color_data).cuda(), flow_grid_before, mode='nearest')
                        rotate_depth = F.grid_sample(Variable(input_depth_data).cuda(), flow_grid_before, mode='nearest')
                    else:
                        rotate_color = F.grid_sample(Variable(input_color_data), flow_grid_before, mode='nearest')
                        rotate_depth = F.grid_sample(Variable(input_depth_data), flow_grid_before, mode='nearest')

                    interm_push_color_feat = self.push_color_trunk.features(rotate_color)
                    interm_push_depth_feat = self.push_depth_trunk.features(rotate_depth)
                    interm_push_feat = torch.cat((interm_push_color_feat, interm_push_depth_feat), dim=1)
                    push_interm_feat.append(interm_push_feat)
                    affine_mat_after = np.asarray([[np.cos(rotate_theta), np.sin(rotate_theta), 0],[-np.sin(rotate_theta), np.cos(rotate_theta), 0]])
                    affine_mat_after.shape = (2,3,1)
                    affine_mat_after = torch.from_numpy(affine_mat_after).permute(2,0,1).float()
                    if self.use_cuda:
                        flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False).cuda(), interm_push_feat.data.size())
                    else:
                        flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False), interm_push_feat.data.size())
                    
                    push_output_prob.append([nn.Upsample(scale_factor=16, mode='bilinear').forward(F.grid_sample(self.pushnet(interm_push_feat), flow_grid_after, mode='nearest'))]) 

            return push_output_prob, push_interm_feat

        else:
            self.push_output_prob = []
            self.push_interm_feat = []

            rotate_idx = specific_rotation 
            rotate_theta = np.radians(rotate_idx*(360/self.num_rotations))

            affine_mat_before = np.asarray([[np.cos(-rotate_theta), np.sin(-rotate_theta), 0],[-np.sin(-rotate_theta), np.cos(-rotate_theta), 0]])
            affine_mat_before.shape = (2,3,1)
            affine_mat_before = torch.from_numpy(affine_mat_before).permute(2,0,1).float()
            if self.use_cuda:
                flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False).cuda(), input_color_data.size())
            else:
                flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False), input_color_data.size())

            if self.use_cuda:
                rotate_color = F.grid_sample(Variable(input_color_data, requires_grad=False).cuda(), flow_grid_before, mode='nearest')
                rotate_depth = F.grid_sample(Variable(input_depth_data, requires_grad=False).cuda(), flow_grid_before, mode='nearest')
            else:
                rotate_color = F.grid_sample(Variable(input_color_data, requires_grad=False), flow_grid_before, mode='nearest')
                rotate_depth = F.grid_sample(Variable(input_depth_data, requires_grad=False), flow_grid_before, mode='nearest')

            interm_push_color_feat = self.push_color_trunk.features(rotate_color)
            interm_push_depth_feat = self.push_depth_trunk.features(rotate_depth)
            interm_push_feat = torch.cat((interm_push_color_feat, interm_push_depth_feat), dim=1)
            self.push_interm_feat.append(interm_push_feat)
            affine_mat_after = np.asarray([[np.cos(rotate_theta), np.sin(rotate_theta), 0],[-np.sin(rotate_theta), np.cos(rotate_theta), 0]])
            affine_mat_after.shape = (2,3,1)
            affine_mat_after = torch.from_numpy(affine_mat_after).permute(2,0,1).float()
            if self.use_cuda:
                flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False).cuda(), interm_push_feat.data.size())
            else:
                flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False), interm_push_feat.data.size())
            
            self.push_output_prob.append([nn.Upsample(scale_factor=16, mode='bilinear').forward(F.grid_sample(self.pushnet(interm_push_feat), flow_grid_after, mode='nearest'))])
            return self.push_output_prob,  self.push_interm_feat

class Tester_Push():
    def __init__(self, model_file_path,is_testing,workspace_limits):
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.action = UR_Grasp()
        self.spawn = Spawn_object(is_testing,workspace_limits,obj_number=3)
        self.cam_intrinsics = np.array([554.5940455214144, 0.0, 320.5, 0.0, 554.5940455214144, 240.5, 0.0, 0.0, 1.0]).reshape(3,3)
        self.push_model = push_net(use_cuda = True)
        self.push_model.load_state_dict(torch.load(model_file_path))  
        self.push_model = self.push_model.cuda()
        print('Pre-trained model  loaded from: %s' % (model_file_path))
        self.push_model.eval()
        self.action.camera.save_images(color_filename="saved_picture/color{}.png",
                            depth_filename="saved_picture/depth{}.png".format(self.action.camera.counter))
        self.picture_num = 0

if __name__ == '__main__':
    mean_push_num = 0
    num_rotations = 16
    x_min = 0.4-0.1
    x_max = 0.975-0.1
    y_min = -0.2
    y_max = 0.24
    workspace_limits = (x_min-0.1,x_max+0.1,y_min-0.1,y_max+0.1, 0.08, 0.15)
    is_testing = True
    model_file_path = 'src/gjt_ur_moveit_gazebo/env_info/push.pth'
    Tester_push = Tester_Push(model_file_path,is_testing,workspace_limits)
    max_test_scenes= 5   
    scene_count = 1           
    Fail = False                     
    Win = False                   
    success_num = 0            
    push_num = 0          
    push_num_sum = 0    
    max_push_num = 2 * Tester_push.spawn.object_number  
    success_rate = 0     
    object_threshold = 0.10
    pred_pix_threshold =  5          
    last_pix_x = 0                  
    last_pix_y = 0                 
    last_angle = 0  
    if scene_count  <= max_test_scenes:
        push_num = 0        
        while not Win  and  scene_count  <= max_test_scenes:
            print(BLUE + '----------------Testing ---epoch:%d / %d -----------------' % (scene_count,max_test_scenes) + ENDC)
            inside = Tester_push.spawn.if_obj_in_env()
            if not inside:
                Fail = True
            Win = all(dist > object_threshold for dist in Tester_push.spawn.model_on_desk_distances)
            if Fail or Win :
                if Fail:
                    print(RED + 'some objects are pushed out of the workspace!!' + ENDC)
                    rospy.sleep(3)
                if not Fail and  Win:
                    print(YELLOW + 'Successfull to object dispersed !!' + ENDC)
                push_num = 0
                Fail = False
                Win = False
                Tester_push.spawn.reset_env_from_txt()
                inside = Tester_push.spawn.if_obj_in_env()
                if not inside:
                    Fail = True
                Win = all(dist > object_threshold for dist in Tester_push.spawn.model_on_desk_distances)
                
            else:
                print(BLUE+'-----get scene image-----'+ENDC)
                color_img, depth_img = Tester_push.action.move.get_camera_data()
                color_heightmap = cv2.resize(color_img,(224,224),interpolation = cv2.INTER_LINEAR)
                depth_heightmap = cv2.resize(depth_img,(224,224),interpolation = cv2.INTER_LINEAR)

                mask,outline = get_outline_from_depth(depth_img)
                outline = cv2.resize(outline,(224,224),interpolation = cv2.INTER_LINEAR)
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                dilated_outline = cv2.dilate(outline, kernel, iterations=1)
                cv2.imshow("push mask", dilated_outline)
                push_mor_gradient = dilated_outline
                cv2.waitKey(100)
                input_color_data,input_depth_data = get_model_input(color_heightmap,depth_heightmap)
                print(GREEN_BOLD+'-------push_model.forward-------'+ENDC)
                push_output_prob, _ = Tester_push.push_model.forward(input_color_data, input_depth_data)
                for rotate_idx in range(len(push_output_prob)):
                    if rotate_idx == 0:
                        push_predictions = push_output_prob[rotate_idx][0].cpu().data.numpy()[:,0,48:272,48:272]
                    else:
                        push_predictions = np.concatenate((push_predictions, push_output_prob[rotate_idx][0].cpu().data.numpy()[:,0,48:272,48:272]), axis=0)
                push_predictions = push_predictions + 0.1
                push_action_mask = np.array(push_mor_gradient.copy()/255).reshape(1,224,224)
                push_predictions = np.multiply(push_predictions, push_action_mask)
                push, push_predicted_value = policynms_push(push_predictions, num_rotations, depth_heightmap)
                '''---------------Compute 3D position of pixel---------[0] - rotate,[1] - pix_y,[2] - pix_x-------------------'''
                best_pix_x = push[2]
                best_pix_y = push[1]
                best_angle = push[0]*(360.0/num_rotations)
                best_rotation_angle = np.deg2rad(best_angle)
                if abs(best_pix_x - last_pix_x) < pred_pix_threshold  and abs(best_pix_y - last_pix_y) < pred_pix_threshold and best_angle == last_angle:
                    best_pix_x = best_pix_x + random.randint(-5, 5)
                    best_pix_y = best_pix_y + random.randint(-5, 5)
                    best_angle = best_angle +  random.choice([-90,-45,45,90])
                last_pix_x = best_pix_x
                last_pix_y = best_pix_y
                last_angle = best_angle
                show_push_pred(color_heightmap,best_pix_x,best_pix_y,best_angle)
                camera_points = Tester_push.action.get_pos_in_camera(best_pix_x,best_pix_y,Tester_push.cam_intrinsics)
                base_link_pos = Tester_push.action.move.camera_to_base_link_pos(camera_points[0][0],camera_points[0][1],camera_points[0][2])
                print('base_link_pos : {}'.format(base_link_pos))
                push_success = Tester_push.action.push(base_link_pos,best_angle)
                Tester_push.spawn.get_distance_after_push()
                Win = all(dist > object_threshold for dist in Tester_push.spawn.model_on_desk_distances)

                print(GREEN+'***************************'+ENDC)
                print(YELLOW_BOLD+'   Successful object separation: %r' % Win+ENDC)
                inside = Tester_push.spawn.if_obj_in_env()
                print(GREEN_BOLD + ' Objects are in the workspace : {} '.format(inside)+ENDC)
                print(GREEN+'***************************'+ENDC)
                push_num += 1
                print('Number of push: %d  (A maximum of %d pushes is allowed)'%(push_num,max_push_num))

                if Win:
                    success_num +=1
                    push_num_sum += push_num
                    mean_push_num = float(push_num_sum)/success_num
                    push_num = 0        
                    scene_count +=1
                    Fail = False
                    Win = False
                    Tester_push.spawn.reset_env_from_txt()
                    rospy.sleep(4)
                    Tester_push.action.camera.save_images(color_filename="saved_picture/color{}.png",
                        depth_filename="saved_picture/depth{}.png".format(Tester_push.action.camera.counter))
                    print(PINK+"Saving {} image".format(Tester_push.action.camera.counter)+ENDC)

                if push_num ==max_push_num:
                    print(RED + 'A maximum of {} pushes is allowed!!'.format(max_push_num) + ENDC)
                    rospy.sleep(1)
                    push_num = 0        
                    scene_count +=1
                    Fail = False
                    Win = False
                    Tester_push.spawn.reset_env_from_txt()
                    rospy.sleep(4)
                    Tester_push.action.camera.save_images(color_filename="saved_picture/color{}.png",
                        depth_filename="saved_picture/depth{}.png".format(Tester_push.action.camera.counter))
                    print(PINK+"Saving {} image".format(Tester_push.action.camera.counter)+ENDC)
                    
                if not inside:
                    print(RED + 'some objects are pushed out of the workspace!!' + ENDC)
                    rospy.sleep(1)
                    push_num = 0        
                    scene_count +=1
                    Fail = False
                    Win = False
                    Tester_push.spawn.reset_env_from_txt()
                    rospy.sleep(4)
                    Tester_push.action.camera.save_images(color_filename="saved_picture/color{}.png",
                        depth_filename="saved_picture/depth{}.png".format(Tester_push.action.camera.counter))
                    print(PINK+"Saving {} image".format(Tester_push.action.camera.counter)+ENDC)
                    