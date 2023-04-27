#!/usr/bin/env python
# -*- coding: utf-8 -*-
#对视觉识别颜色降低需求，结合3D雷达，只需要知道点云是蓝色或着红色即可，对周边环境的颜色不感兴趣(不怕场地地图的干扰)
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf
import tf.transformations as tf_trans
from tf2_geometry_msgs import PointStamped

class LoopDect:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.listener = tf.TransformListener()
        self.camera_inner_matrix = rospy.get_param("/video_tackle/camera_matrix/data") 
        self.camera_dist_coefs =rospy.get_param("/video_tackle/distortion_coefficients/data")
        self.camera_exteral_matrix = LoopDect.get_exteral_matrix(self) 

        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        # 内置摄像头
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        #　深度摄像头
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.kernel = np.ones((3, 3), np.uint8) 

        
    def camera_callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            bgr_img = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as error:
            rospy.loginfo("error")

        # hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        # red_img = cv2.inRange(hsv_img, (0,133,70), (5,255,255))   
        # blue_img = cv2.inRange(hsv_img, (104,213,26), (160,255,255))
             
        # red_radius = LoopDect.new_find_target(self,red_img,bgr_img,cv_image)
        # blue_radius = LoopDect.new_find_target(self,blue_img,bgr_img,cv_image)

        LoopDect.coordinate_transform(self,cv_image)

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

    def get_exteral_matrix(self):
        # 监听相机坐标系与世界坐标系之间的变换关系
        self.listener.waitForTransform('world', 'camera', rospy.Time(), rospy.Duration(4.0))
        # 获取相机在世界坐标系下的位姿
        (trans, rot) = self.listener.lookupTransform('world', 'camera', rospy.Time())
        R = tf_trans.quaternion_matrix(rot)[:3, :3]
        # 将位姿转换为旋转矩阵和平移向量
        t = np.array(trans)
        # 组合成外参矩阵
        ext_mat = np.hstack((np.array(R), t.reshape(3, 1)))
        return ext_mat

    def coordinate_transform(self,arg1):
        # 相机内参矩阵
        fx = self.camera_inner_matrix[0]
        cx = self.camera_inner_matrix[2]
        fy = self.camera_inner_matrix[4]
        cy = self.camera_inner_matrix[5]
        inner_matrix = np.array([[fx, 0, cx],
                                [0, fy, cy],
                                [0, 0, 1]])
        # 相机畸变参数
        k1 = self.camera_dist_coefs[0]
        k2 = self.camera_dist_coefs[1]
        p1 = self.camera_dist_coefs[2]
        p2 = self.camera_dist_coefs[3]
        k3 = self.camera_dist_coefs[4]
        dist_coefs = np.array([k1, k2, p1, p2, k3])

        # # 相机外参矩阵，可以通过标定得到，也可以手动设置
        # R = np.array([[r11, r12, r13],
        #             [r21, r22, r23],
        #             [r31, r32, r33]])
        # T = np.array([[t1], [t2], [t3]])
        # external_matrix = np.concatenate((R, T), axis=1)
        external_matrix = self.camera_exteral_matrix

        # 世界坐标系中的点
        world_point = np.array([[100], [0], [0], [1]])
        # world_point = np.array([[x], [y], [z], [1]])

        # # 将世界坐标系中的点转换为相机坐标系中的点
        # 相机坐标系的 x, y, z 轴通常是和相机的水平方向、垂直方向、深度方向（即相机拍摄的方向）对齐
        # # 将相机坐标系中的点转换为像素坐标系中的点
        camera_point = np.dot(external_matrix, world_point)

        pixel_point = np.array(np.dot(inner_matrix, camera_point[:3]) / camera_point[2])
        pixel_point = np.transpose(pixel_point[:2])
        pixel_point = np.array(pixel_point,dtype=np.float32)
        # 畸变矫正
        pixel_point = cv2.undistortPoints(pixel_point, inner_matrix, dist_coefs)
        # 提取像素坐标左上角为原点，向右为X轴正方向，向下为Y轴正方向
        # print(pixel_point)
        u, v = pixel_point[0][0] 
        u += 320
        v += 240
        # print(u)
        # print(v)
        # 在图像上绘制点
        cv2.circle(arg1, (int(u), int(v)), 50, (0, 0, 255), -1)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(arg1, "bgr8"))
        # cv2.imshow("image", img)

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








