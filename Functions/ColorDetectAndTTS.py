#!/usr/bin/python3
# coding=utf8
import sys
import import_path
import cv2
import math
import time
import Camera
import threading
import numpy as np
import yaml_handle
import HiwonderSDK.TTS as TTS
import HiwonderSDK.Misc as Misc
import kinematics as kinematics
import HiwonderSDK.Board as Board

# 颜色检测

tts = TTS.TTS()
ik = kinematics.IK()

HWSONAR = None
debug = False

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0

    area_max_contour = None
    max_area = 0

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 100:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓, 面积

# 初始位置
def initMove():
    Board.setPWMServoPulse(1, 1500, 500)
    Board.setPWMServoPulse(2, servo_data['servo2'], 500)

color_list = []
detect_color = 'None'
action_finish = True
draw_color = range_rgb["black"]
# 变量重置
def reset():
    global draw_color
    global color_list
    global detect_color
    global action_finish
    
    color_list = []
    detect_color = 'None'
    action_finish = True
    draw_color = range_rgb["black"]
    
# app初始化调用
def init():
    print("ColorDetect Init")
    load_config()
    initMove()

__isRunning = False
# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorDetect Start")

# app停止玩法调用
def stop():
    global __isRunning
    __isRunning = False
    print("ColorDetect Stop")

# app退出玩法调用
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("ColorDetect Exit")

color_dict = {'red': '红色',
        'green': '绿色',
        'blue': '蓝色'
        }

def move():
    global draw_color
    global detect_color
    global action_finish
    
    speaker = '[m3]'
    while True:
        if debug:
            return
        if __isRunning:
            if detect_color != 'None':
                if detect_color == 'red':
                    speaker = '[m3]'
                elif detect_color == 'green':
                    speaker = '[m51]'
                elif detect_color == 'blue':
                    speaker = '[m55]'
                tts.TTSModuleSpeak(speaker, '检测到' + color_dict[detect_color])
                time.sleep(2)
                detect_color = 'None'
                draw_color = range_rgb["black"]
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

size = (320, 240)
def run(img):
    global draw_color
    global color_list
    global detect_color
    global action_finish
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    if action_finish:
        for i in lab_data:
            if i != 'black' and i != 'white':
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀
                if debug:
                    cv2.imshow(i, dilated)
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
                areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 100:  # 有找到最大面积
            ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆
            centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
            centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))            
            cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)#画圆

            if color_area_max == 'red':  #红色最大
                color = 1
            elif color_area_max == 'green':  #绿色最大
                color = 2
            elif color_area_max == 'blue':  #蓝色最大
                color = 3
            else:
                color = 0
            color_list.append(color)

            if len(color_list) == 5:  #多次判断
                # 取平均值
                color = int(round(np.mean(np.array(color_list))))
                color_list = []
                if color == 1:
                    detect_color = 'red'
                    draw_color = range_rgb["red"]
                elif color == 2:
                    detect_color = 'green'
                    draw_color = range_rgb["green"]
                elif color == 3:
                    detect_color = 'blue'
                    draw_color = range_rgb["blue"]
                else:
                    detect_color = 'None'
                    draw_color = range_rgb["black"]               
        else:
            detect_color = 'None'
            draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img

if __name__ == '__main__':
    import HiwonderSDK.Sonar as Sonar
    from CameraCalibration.CalibrationConfig import *
    
    #加载参数
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    
    debug = False
    if debug:
        print('Debug Mode')
    
    HWSONAR = Sonar.Sonar()
    init()
    start()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    ik = kinematics.IK()
    ik.stand(ik.initial_pos)
    tts.TTSModuleSpeak('[h0][v10][m3]', '我准备好了')
    time.sleep(1)
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()