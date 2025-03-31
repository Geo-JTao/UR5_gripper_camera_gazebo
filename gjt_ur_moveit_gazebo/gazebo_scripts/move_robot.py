#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import  PlanningScene, ObjectColor,CollisionObject, AttachedCollisionObject,Constraints,OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
import numpy as np
import warnings
warnings.filterwarnings("ignore", message="ABORTED: Solution found but controller failed during execution")
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math,cv2

class MoveIt_Control:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_control_server', anonymous=False)
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm.set_goal_joint_tolerance(0.01)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)

        self.end_effector_link = self.arm.get_end_effector_link()
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        self.arm.set_planner_id("TRRT")
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)
        self.go_home()
        self.set_scene()  
        self.setup_sim_camera()

    def euler2rotm(self,theta):
        # Get rotation matrix from euler angles
        R_x = np.array([[1,         0,                  0                   ],
                        [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                        [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                        ])
        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])         
        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])            
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R
    def setup_sim_camera(self):
        # rosrun tf tf_echo /world /camera_link_optical   
        # in RPY (radian) [-3.141, 0.000, -0.001]
        cam_position =  [0.610, 0.000, 0.750]
        cam_orientation = [-3.141, 0.000, -0.001]
        cam_trans = np.eye(4,4)
        cam_trans[0:3,3] = np.asarray(cam_position)
        cam_orientation = [cam_orientation[0], cam_orientation[1], cam_orientation[2]]
        cam_rotm = np.eye(4,4)
        cam_rotm[0:3,0:3] = np.linalg.inv(self.euler2rotm(cam_orientation))
        self.cam_pose = np.dot(cam_trans, cam_rotm)
        #  rostopic echo /camera/rgb/camera_info 
        self.cam_intrinsics = np.asarray([[554.5940455214144, 0.0, 320.5], [0.0, 554.5940455214144, 240.5], [0, 0, 1]])
        self.cam_depth_scale = 1
        return self.cam_pose,self.cam_intrinsics,self.cam_depth_scale
    
    def camera_to_base_link_pos(self,x,y,z):
        cam_pos = np.array([x,y,z,1])
        tran_R_1 = np.dot(self.cam_pose[0:3,0:3],cam_pos[0:3])
        tran_T_2 = tran_R_1 + self.cam_pose[0:3,3]
        base_link_pos = tran_T_2
        return base_link_pos

    def get_camera_data(self):
        rospy.sleep(1)
        raw_img = rospy.wait_for_message('/camera/rgb/image_raw', Image)
        color_img = CvBridge().imgmsg_to_cv2(raw_img, "bgr8")
        depth_img = rospy.wait_for_message('/camera/depth/image_raw', Image)
        rospy.sleep(0.01)
        depth_image = CvBridge().imgmsg_to_cv2(depth_img,desired_encoding='passthrough') 
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_img= cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
        return color_img, depth_img

    def set_scene(self):
        self.scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        rospy.sleep(1)
        table_id = 'table'
        self.scene.remove_world_object(table_id)
        rospy.sleep(1)
        table_size = [1.5,1.5,0.1]
        table_pose = PoseStamped()
        table_pose.header.frame_id = "world"
        table_pose.pose.position.x = 0.01
        table_pose.pose.position.y = 0.65
        table_pose.pose.position.z = 0
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box(table_id, table_pose, table_size)
        self.setColor(table_id, 0.5, 0.5, 0.5, 1.0)
        self.sendColors()

    def move_j(self, joint_configuration=None,a=1,v=1,time_sleep = 0.5):
        joint_value = self.arm.get_current_joint_values()
        if joint_configuration==None:
            joint_configuration = [-1.4536550680743616, -1.3324196974383753,  -1.2551777998553675, 
                                    -1.7819412390338343, 1.588425874710083, 0.10336843878030777]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        self.arm.set_joint_value_target(joint_configuration)
        self.arm.go()
        rospy.sleep(time_sleep)

    def move_p(self, position,RPY= [0.5082812350013831, 1.3579148036462245, 2.1664518919017026],a=1,v=1):
        if position is None:
            print("you need input the position ")
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2] 
        q = quaternion_from_euler(RPY[0],RPY[1],RPY[2])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(0.5)

    def go_home(self,a=1,v=1):
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        grasp_home= [ 0.000196635662551, -1.75733057251, 1.30864952802,
                                      -1.11879747439, -1.5668002396, -1.54]
        self.move_j(grasp_home)
        rospy.sleep(0.3)


    def setColor(self, name, r, g, b, a=0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

    def testRobot(self):
        try:
            print("Test for robot...")
            self.go_home()
            x = 0.369009573269
            y =0.109407895568
            z= 0.474318830582
            position = [x,y,z]
            self.move_p(position)
            rospy.sleep(1)
            self.go_home()
        except:
            print("Test fail! ")


if __name__ =="__main__":
    moveit_server = MoveIt_Control()
    moveit_server.testRobot()