#此程序用来调节颜色参数
import cv2
import numpy as np

def on_bgr_value_change(args):
    # 获取img_bgr窗口上的名为lower_b滑动条的当前值
    lower_h_value = cv2.getTrackbarPos('lower_h', 'hsv_img')
    # 获取img_bgr窗口上的名为upper_b滑动条的当前值
    upper_h_value = cv2.getTrackbarPos('upper_h', 'hsv_img')
    # 获取img_bgr窗口上的名为lower_g滑动条的当前值
    lower_s_value = cv2.getTrackbarPos('lower_s', 'hsv_img')
    # 获取img_bgr窗口上的名为upper_g滑动条的当前值
    upper_s_value = cv2.getTrackbarPos('upper_s', 'hsv_img')
    # 获取img_bgr窗口上的名为lower_r滑动条的当前值
    lower_v_value = cv2.getTrackbarPos('lower_v', 'hsv_img')
    # 获取img_bgr窗口上的名为upper_r滑动条的当前值
    upper_v_value = cv2.getTrackbarPos('upper_v', 'hsv_img')

    bgr_divide_binary = cv2.inRange(hsv_img, (lower_h_value, lower_s_value, lower_v_value),
                                    (upper_h_value, upper_s_value, upper_v_value))
    cv2.namedWindow('bgr_divide_binary', cv2.WINDOW_NORMAL)
    cv2.imshow('bgr_divide_binary', bgr_divide_binary)  

def find_target(arg1,arg2):
    cv2.imshow('Initial', arg1)
    kernel = np.ones((5, 5), np.uint8)
    eroding = cv2.morphologyEx(arg1, op=cv2.MORPH_ERODE, kernel=kernel, iterations=2)
    dilating = cv2.morphologyEx(eroding, op=cv2.MORPH_DILATE, kernel=kernel, iterations=2)
    # one_channel = cv2.cvtColor(closing, cv2.COLOR_BGR2GRAY)
    # one_channel = cv2.cvtColor(arg1, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Red', dilating)
    contours, hierarchy = cv2.findContours(dilating, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours)!=0:
        diameter_list = []
        center_idx_list = []

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
        print(max(diameter_list))
        target_diameter_index = diameter_list.index(max(diameter_list))
        # print(target_diameter_index)
        # print(center_idx_list[target_diameter_index][0],center_idx_list[target_diameter_index][1])
        cv2.drawContours(arg2, contours, idx, (0,0,255), 3)
        cv2.ellipse(arg2, box, (255,0,0), 2)
        cv2.circle(arg2, center_idx_list[target_diameter_index], (int)(max(diameter_list)/2),(0,255,0), 5)
        cv2.putText(arg2, '(%d, %d)' % (center_idx_list[target_diameter_index][0],center_idx_list[target_diameter_index][1]),center_idx_list[target_diameter_index],cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)

if __name__ == '__main__':

    cv2.namedWindow('RGB_img', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
    cv2.namedWindow('Red', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
    cv2.namedWindow('Initial', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
    ask = 0
    img = cv2.imread("/home/ubuntu/wtr_upmachine_ws/src/visual_dect/photo/photo/red_loop_debug.jpg")
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_img = cv2.inRange(hsv_img, (0,133,70), (5,255,255))
    
    
    red_radius = find_target(red_img,img)
    cv2.imshow('RGB_img', img)
    # red_img = cv2.inRange(hsv_img, (0,125,49), (14,255,255))
    # cv2.imshow("i",red_img)
    # filename ="/home/ubuntu/demo02_ws/src/Task2/photo/photo/red_loop_hsv.jpg"
    # cv2.imwrite(filename, hsv_img)

    # 调节识别颜色参数
    if(ask==1):
        # hsv_img = cv2.imread('/home/ubuntu/demo02_ws/src/Task2/photo/photo/red_loop_hsv.jpg')
        cv2.namedWindow('hsv_img', cv2.WINDOW_NORMAL) 
        # 在img_bgr窗口上创建lower_b进度条,范围为0-179,回调函数为on_bgr_value_change
        cv2.createTrackbar('lower_h', 'hsv_img', 0, 255, on_bgr_value_change)
        # 在img_bgr窗口上创建upper_b进度条,范围为0-179,回调函数为on_bgr_value_change
        cv2.createTrackbar('upper_h', 'hsv_img', 0, 255, on_bgr_value_change)
        # 在img_bgr窗口上创建lower_g进度条,范围为0-255,回调函数为on_bgr_value_change
        cv2.createTrackbar('lower_s', 'hsv_img', 0, 255, on_bgr_value_change)
        # 在img_bgr窗口上创建upper_g进度条,范围为0-255,回调函数为on_bgr_value_change
        cv2.createTrackbar('upper_s', 'hsv_img', 0, 255, on_bgr_value_change)
        # 在img_bgr窗口上创建lower_r进度条,范围为0-255,回调函数为on_bgr_value_change
        cv2.createTrackbar('lower_v', 'hsv_img', 0, 255, on_bgr_value_change)
        # 在img_bgr窗口上创建upper_r进度条,范围为0-255,回调函数为on_bgr_value_change
        cv2.createTrackbar('upper_v', 'hsv_img', 0, 255, on_bgr_value_change)
        cv2.imshow('hsv_img', hsv_img)

    cv2.waitKey(0)
    if cv2.waitKey(1) == ord('q'):  # 如果按键输入q
        cv2.destroyAllWindows()  # 销毁所有窗口


