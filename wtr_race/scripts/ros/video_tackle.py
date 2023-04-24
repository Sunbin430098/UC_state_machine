#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
        # blue_img = cv2.inRange(hsv_img, (104,213,26), (160,255,255))
             
        red_radius = LoopDect.new_find_target(self,red_img,bgr_img,cv_image)
        # blue_radius = LoopDect.new_find_target(self,blue_img,bgr_img,cv_image)

        
        # find_min(red_radius,blue_radius,bgr_img)

    def new_find_target(self,arg1,arg2,arg3):
        # cv2.namedWindow('bgr_img', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        # eroding = cv2.morphologyEx(arg1, op=cv2.MORPH_ERODE, kernel=kernel, iterations=2)
        dilating = cv2.morphologyEx(arg1, op=cv2.MORPH_DILATE, kernel=self.kernel, iterations=3)
        contours, hierarchy = cv2.findContours(dilating, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)!=0:
            diameter_list = []
            center_idx_list = []
            flag = 0
            for idx, contour in enumerate(contours):
                area = cv2.contourArea(contours[idx])
                if len(contours[idx])<5:
                    continue
                box = center_idx, size_idx, angle_idx = cv2.fitEllipse(contours[idx])
                if max(size_idx[0], size_idx[1]) < min(size_idx[0], size_idx[1]) * 1.3:
                    center_idx = np.uint16(center_idx)
                    size_idx = np.uint16(size_idx)
                    diameter = np.mean(size_idx)
                    diameter_list.append(diameter)
                    center_idx_list.append(center_idx)
                    if len(diameter_list)!=0  :
                        if max(diameter_list)>150:
                            flag = 1
                            target_diameter_index = diameter_list.index(max(diameter_list))
                            # cv2.drawContours(arg2, contours, idx, (0,0,255), 3)
                            # cv2.ellipse(arg2, box, (255,0,0), 2)
                            cv2.circle(arg3, (center_idx_list[target_diameter_index][0],center_idx_list[target_diameter_index][1]), (int)(max(diameter_list)/2),(0,255,0), 5)
                            cv2.putText(arg3, "center" ,(center_idx_list[target_diameter_index][0],center_idx_list[target_diameter_index][1]),cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)
                             
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(arg3, "bgr8"))
            if flag == 1:
                print(max(diameter_list))
                print(target_diameter_index)
                print(center_idx_list[target_diameter_index][0],center_idx_list[target_diameter_index][1])
                return max(diameter_list)    
        else :
            diameter = 10000
            return diameter

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