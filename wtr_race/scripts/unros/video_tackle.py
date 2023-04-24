#视频流标记色球逻辑:
#   读取视频流图像video_img实时转成HSV,inRange函数进行分割(此时应该能分割四次得到四个颜色的hsv三通道)
#   分别调用fing_target函数进行色球轮廓提取(可以反馈到video_img标记出轮廓)
#   测距离，最后只留下距离最近的那个(目前的想法是半径最小的那个__python函数有返回值)
#   深度摄像头使用网站：https://www.ncnynl.com/archives/201703/1444.html

import cv2
import numpy as np

# def blue_mask_operation(hsv_img):

    # 取交集

    # return mask

def red_mask_operation(hsv_img):
    # 定义红色范围
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    # 过滤出红色区域
    red_mask1 = cv2.inRange(hsv_img, lower_red, upper_red)
    red_mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
    red_img = red_mask1 + red_mask2
    return red_img


def find_min(red_radius,blue_radius,bgr_img):
    return 1

def new_find_target(arg1,arg2):
    kernel = np.ones((3, 3), np.uint8)
    eroding = cv2.morphologyEx(arg1, op=cv2.MORPH_ERODE, kernel=kernel, iterations=2)
    dilating = cv2.morphologyEx(arg1, op=cv2.MORPH_DILATE, kernel=kernel, iterations=3)
    # one_channel = cv2.cvtColor(closing, cv2.COLOR_BGR2GRAY)
    # one_channel = cv2.cvtColor(arg1, cv2.COLOR_BGR2GRAY)
    # contours, hierarchy = cv2.findContours(dilating, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # contours, hierarchy = cv2.findContours(dilating, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(dilating, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
            if radius >100  :
                cv2.circle(arg2, center, int(radius), (0, 255, 255), 2)
                print(radius)
        #     box = center_idx, size_idx, angle_idx = cv2.fitEllipse(contours[idx])
        #     if max(size_idx[0], size_idx[1]) < min(size_idx[0], size_idx[1]) * 1.3:
        #         center_idx = np.uint16(center_idx)
        #         size_idx = np.uint16(size_idx)
        #         diameter = np.mean(size_idx)
        #         diameter_list.append(diameter)
        #         center_idx_list.append(center_idx)
        #         if len(diameter_list)!=0  :
        #             if max(diameter_list)>150:
        #                 flag = 1
        #                 target_diameter_index = diameter_list.index(max(diameter_list))
        #                 # cv2.drawContours(arg2, contours, idx, (0,0,255), 3)
        #                 # cv2.ellipse(arg2, box, (255,0,0), 2)     
        #                 cv2.circle(arg2, center_idx_list[target_diameter_index], (int)(max(diameter_list)/2),(0,255,0), 5)
        #                 cv2.putText(arg2, '(%d, %d)' % (center_idx_list[target_diameter_index][0],center_idx_list[target_diameter_index][1]),center_idx_list[target_diameter_index],cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)
                        
        cv2.imshow('bgr_img', arg2)
        
        if flag == 1:
            # print(max(diameter_list))
            # print(target_diameter_index)
            # print(center_idx_list[target_diameter_index][0],center_idx_list[target_diameter_index][1])
            return max(diameter_list)    
    else :
        diameter = 10000
        return diameter

if __name__ == '__main__':
    cv2.namedWindow('bgr_img', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

    cap = cv2.VideoCapture(0)
    kernel = np.ones((3, 3), np.uint8) 

    while cap.isOpened():    
        ret, bgr_img = cap.read()
        hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        # red_img = cv2.inRange(hsv_img, (0,133,70), (5,255,255))
        # red_dilating = cv2.morphologyEx(red_img, op=cv2.MORPH_DILATE, kernel=kernel, iterations=3)
        red_img = red_mask_operation(hsv_img)

        blue_img = cv2.inRange(hsv_img, (104,213,26), (160,255,255))
        # blue_img = blue_mask_operation(hsv_img)
        # blue_dilating = cv2.morphologyEx(blue_img, op=cv2.MORPH_DILATE, kernel=kernel, iterations=3)
 
        
        red_radius = new_find_target(red_img,bgr_img)
        blue_radius = new_find_target(blue_img,bgr_img)
        # find_min(red_radius,blue_radius,bgr_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            break

    cv2.destroyAllWindows()
