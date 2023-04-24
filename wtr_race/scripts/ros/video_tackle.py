#!/usr/bin/env python
# -*- coding: utf-8 -*-
#对视觉识别颜色降低需求，结合3D雷达，只需要知道点云是蓝色或着红色即可，对周边环境的颜色不感兴趣(不怕场地地图的干扰)
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class LoopDect:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        # 内置摄像头
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        #　深度摄像头
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.kernel = np.ones((3, 3), np.uint8) 
        
    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            bgr_img = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as error:
            rospy.loginfo("error")

        hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        red_img = cv2.inRange(hsv_img, (0,133,70), (5,255,255))   
        blue_img = cv2.inRange(hsv_img, (104,213,26), (160,255,255))
             
        red_radius = LoopDect.new_find_target(self,red_img,bgr_img,cv_image)
        blue_radius = LoopDect.new_find_target(self,blue_img,bgr_img,cv_image)


    def new_find_target(self,arg1,arg2,arg3):
        dilating = cv2.morphologyEx(arg1, op=cv2.MORPH_DILATE, kernel=self.kernel, iterations=3)
        contours, hierarchy = cv2.findContours(dilating, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)!=0:
            diameter_list = []
            center_idx_list = []
            flag = 0
            for idx, contour in enumerate(contours):
                if len(contours[idx])<5:
                    continue
                area = cv2.contourArea(contours[idx])
                if area < 100:
                    continue
                (x, y), radius = cv2.minEnclosingCircle(contours[idx])
                center = (int(x), int(y))
                if radius < 50 :
                    continue
                if radius >50 :
                    cv2.circle(arg3, center, int(radius), (0, 255, 255), 2)
                    print(radius)            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(arg3, "bgr8"))
        else :
            diameter = 10000
            return diameter
    def ProjectTransform(self,arg1):
        # 定义相机内参矩阵
        K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]])
        # 定义相机外参矩阵
        R = np.array([[r11, r12, r13],
              [r21, r22, r23],
              [r31, r32, r33]])
        t = np.array([[t1],
                    [t2],
                    [t3]])
        # 定义三维坐标点
        point_3d = np.array([[x],
                            [y],
                            [z]])

        # 将三维坐标点投影到像素坐标系中
        point_2d, _ = cv2.projectPoints(point_3d, R, t, K, None)

    def cleanup(self):
        rospy.loginfo("Shutting down vision node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node("visual_dect")
        LoopDect()
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()