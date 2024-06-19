#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/SpiderPi/')
import cv2
import math
import time
import Camera
import numpy as np
import HiwonderSDK.Board as Board

# 人脸检测

# 阈值
conf_threshold = 0.6

# 模型位置
modelFile = "/home/pi/SpiderPi/models/res10_300x300_ssd_iter_140000_fp16.caffemodel"
configFile = "/home/pi/SpiderPi/models/deploy.prototxt"
net = cv2.dnn.readNetFromCaffe(configFile, modelFile)

size = (320, 240)
def run(img):          
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    blob = cv2.dnn.blobFromImage(img_copy, 1, (150, 150), [104, 117, 123], False, False)
    net.setInput(blob)
    detections = net.forward() #计算识别
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold:
            #识别到的人了的各个坐标转换会未缩放前的坐标
            x1 = int(detections[0, 0, i, 3] * img_w)
            y1 = int(detections[0, 0, i, 4] * img_h)
            x2 = int(detections[0, 0, i, 5] * img_w)
            y2 = int(detections[0, 0, i, 6] * img_h)             
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2, 8) #将识别到的人脸框出
            
            Board.setBuzzer(0) # 关闭
            Board.setBuzzer(1) # 打开
            time.sleep(0.3) # 延时
            Board.setBuzzer(0) #关闭    

    return img

if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import *
    
    #加载参数
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)    
    
    
    my_camera = Camera.Camera()
    my_camera.camera_open()
    
    print("Face_Detect Init")
    print("Face_Detect Start")
    
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
