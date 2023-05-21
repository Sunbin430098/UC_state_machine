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
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)
        #　深度摄像头
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.kernel = np.ones((3, 3), np.uint8) 
        self.count = 0  
        self.red_y = 0
        self.blue_y = 0
        
    def camera_callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            self.bgr_img = np.array(self.cv_image, dtype=np.uint8)
        except CvBridgeError as error:
            rospy.loginfo("error")
             
        # red_radius = LoopDect.new_find_target(self,red_img,bgr_img,cv_image)
        # blue_radius = LoopDect.new_find_target(self,blue_img,bgr_img,cv_image)
        self.lidar_ask_camera = rospy.get_param("/decay_map_test/lidar_ask_camera",1)
        self.lidar_ask_camera_x = rospy.get_param("/decay_map_test/lidar_ask_camera",1)
        self.lidar_ask_camera_y = rospy.get_param("/decay_map_test/lidar_ask_camera",0)
        self.lidar_ask_camera_z = rospy.get_param("/decay_map_test/lidar_ask_camera",0)
        if self.lidar_ask_camera == 1:
            camera_call_lidar = LoopDect.coordinate_transform(self)
            rospy.set_param("call_lidar",camera_call_lidar)

    def transform_find_target(self):#直接比较最上面的环颜色，对摄像头要求太高
        side_length = 200
        mask = np.zeros_like(self.bgr_img)
        cv2.rectangle(mask, (int(self.u), int(self.v)), (int(self.u) + side_length, int(self.v) + side_length), (255, 255, 255), -1)
        
        # red_result = cv2.bitwise_and(self.red_img, mask)
        # blue_result = cv2.bitwise_and(self.blue_img, mask)
        self.result = cv2.bitwise_and(self.bgr_img, mask)
        self.hsv_img = cv2.cvtColor(self.result, cv2.COLOR_BGR2HSV)
        self.red_img = cv2.inRange(self.hsv_img, (0,133,70), (5,255,255))   
        self.blue_img = cv2.inRange(self.hsv_img, (104,213,26), (160,255,255))

        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.result, "bgr8"))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.red_img, "8UC1"))
        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.blue_img, "8UC1"))

        # gray = cv2.cvtColor(self.result, cv2.COLOR_BGR2GRAY)
        # # 进行圆形霍夫变换检测圆环
        # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=50, param2=30, minRadius=10, maxRadius=50)
        
        red_circles = cv2.HoughCircles(self.red_img, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=50, param2=30, minRadius=10, maxRadius=50)
        blue_circles = cv2.HoughCircles(self.blue_img, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=50, param2=30, minRadius=10, maxRadius=50)
        
        if red_circles is not None:
            red_circles = sorted(red_circles, key=lambda x: x[1])
            top_red_circle = red_circles[0]
            red_x, self.red_y, red_r = top_red_circle
        elif blue_circles is not None:
            blue_circles = sorted(blue_circles, key=lambda x: x[1])
            top_blue_circle = blue_circles[0]
            blue_x, self.blue_y, blue_r = top_blue_circle
        if self.red_y<self.blue_y:
            print("颜色为红色")
            return 1
        else:
            print("颜色为蓝色")
            return 0

    def new_find_target(self,arg1):
        dilating = cv2.morphologyEx(arg1, op=cv2.MORPH_DILATE, kernel=self.kernel, iterations=3)
        contours, hierarchy = cv2.findContours(dilating, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)!=0:
            for idx, contour in enumerate(contours):
                if len(contours[idx])<5:
                    continue
                area = cv2.contourArea(contours[idx])
                if area < 10:
                    continue
                (x, y), radius = cv2.minEnclosingCircle(contours[idx])
                center = (int(x), int(y))
                if radius < 10 :
                    continue
                if radius >10 :
                    cv2.circle(self.cv_image, center, int(radius), (0, 255, 255), 2)
                    print(radius)            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        else :
            diameter = 10000
            return diameter

    def get_exteral_matrix(self):#外参矩阵实时获取
        self.listener.waitForTransform('world', 'camera_rgb_frame', rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = self.listener.lookupTransform('world', 'camera_rgb_frame', rospy.Time())
        R = tf_trans.quaternion_matrix(rot)[:3, :3]
        t = np.array(trans)
        ext_mat = np.hstack((np.array(R), t.reshape(3, 1)))
        return ext_mat

    def coordinate_transform(self):#三维世界转二维像素的坐标变换
        self.count +=1
        fx = self.camera_inner_matrix[0]
        cx = self.camera_inner_matrix[2]
        fy = self.camera_inner_matrix[4]
        cy = self.camera_inner_matrix[5]
        inner_matrix = np.array([[fx, 0, cx],
                                [0, fy, cy],
                                [0, 0, 1]])
        k1 = self.camera_dist_coefs[0]
        k2 = self.camera_dist_coefs[1]
        p1 = self.camera_dist_coefs[2]
        p2 = self.camera_dist_coefs[3]
        k3 = self.camera_dist_coefs[4]
        dist_coefs = np.array([k1, k2, p1, p2, k3])

        external_matrix = self.camera_exteral_matrix
        # print(external_matrix)
        # print(inner_matrix)

        #世界转相机         #Astra相机的坐标系，坐标系原点位于相机光学中心：
                            # X轴：指向图像右侧
                            # Y轴：指向图像下方(yz改好了和世界逻辑相同)
                            # Z轴：指向相机前方
                            # x轴值应保持半米以上,视角不宽
        world_point = np.array([[self.lidar_ask_camera_x], [self.lidar_ask_camera_y], [self.lidar_ask_camera_z], [1]])
        # world_point = np.array([[1.0], [0.445], [0.4], [1]])
        camera_point = np.array(np.dot(external_matrix, world_point))
        # print(camera_point) 
        camera_point = camera_point/abs(camera_point[2])

        # 相机转像素
        pixel_point = np.array(np.dot(inner_matrix, camera_point) )
        pixel_point = np.transpose(pixel_point[:2])
        pixel_point[0][0] = 640-pixel_point[0][0]
        pixel_point[0][1] = 480-pixel_point[0][1]
        # print(pixel_point)

        # # 畸变矫正
        # pixel_point = cv2.undistortPoints(pixel_point, inner_matrix, dist_coefs,normalize = False)
        # print(pixel_point)
        # u, v = (pixel_point[0][0]) 
        # u *= 640
        # v *= 480

        self.u, self.v = (pixel_point[0]) 
        if self.count %20 == 0:
            self.count = 0
            # print(self.u)
            # print(self.v)
        # cv2.circle(arg1, (int(u), int(v)), 50, (0, 0, 255), -1)
        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(arg1, "bgr8"))
            is_target_loop = LoopDect.new_find_target(self,self.red_img)
            # is_target_loop = LoopDect.transform_find_target(self)
            if is_target_loop > 0:
                return 1
            else: return 0
        else : return 0

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








