#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
import os

if not os.path.exists(''):
    with open('src/picture_counter.txt','w') as f:
        f.write('0')

class ImageSaver:
    def __init__(self):
        self.color_image = None
        self.depth_image = None
        self.bridge = cv_bridge.CvBridge()
        self.counter = 0
        # rospy.init_node('image_saver')
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        
    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        # self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(self.depth_image, dtype=np.float32)
        self.depth_image = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
        
    def inpaint(img, missing_value=0):
        img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        mask = (img == missing_value).astype(np.uint8)
        scale = np.abs(img).max()
        img = img.astype(np.float32) / scale  
        img = cv2.inpaint(img, mask, 1, cv2.INPAINT_NS)
        img = img[1:-1, 1:-1]
        img = img * scale
        return img  

    def save_images(self, color_filename="color.jpg", depth_filename="depth.jpg"):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            if self.color_image is not None and self.depth_image is not None:
                break
            rate.sleep()
        cv2.imwrite(color_filename.format(self.counter), self.color_image)
        cv2.imwrite(depth_filename.format(self.counter), self.depth_image)
        
        with open('src/picture_counter.txt','r+') as f:
            f.seek(0)
            f.write(str(self.counter))
        self.counter += 1
        return self.color_image,self.depth_image

if __name__ == '__main__':
    image_saver = ImageSaver()
    image_saver.save_images(color_filename="saved_picture/color{}.png",
                            depth_filename="saved_picture/depth{}.png".format(image_saver.counter))
    print("Saving {} image".format(image_saver.counter))


