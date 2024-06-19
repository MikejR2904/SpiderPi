#!/usr/bin/python3
# coding=utf8
import os
import sys
sys.path.append('/home/pi/SpiderPi/')
import time
import math
import HiwonderSDK.Board as Board
import HiwonderSDK.Mpu6050 as Mpu6050
import HiwonderSDK.ActionGroupControl as AGC
import Functions.kinematics as kinematics

Board.setBuzzer(1)
time.sleep(0.1)
Board.setBuzzer(0)

ik = kinematics.IK()
ik.stand(ik.initial_pos)

# 姿态检测
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
********功能:幻尔科技SpiderPi 姿态检测例程*********
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
以下指令均需在LX终端使用，LX终端可通过ctrl+alt+t打开，或点
击上栏的黑色LX终端图标。
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

if __name__ == '__main__':
    
    count1 = 0
    count2 = 0
    control_lock = False
    #mpu6050初始化
    mpu = Mpu6050.mpu6050(0x68)
    mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)
    mpu.set_accel_range(mpu.ACCEL_RANGE_2G)
    
    while True:
        accel_date = mpu.get_accel_data(g=True)  # 获取传感器值
        angle_x = int(math.degrees(math.atan2(accel_date['x'], accel_date['z'])))  # 转化为角度值
       
        if abs(angle_x) > 130:
            count1 += 1
        else:
            count1 = 0
            
        if abs(angle_x) < 50:
            count2 += 1
        else:
            count2 = 0
        time.sleep(0.1)
        
        if count1 >= 3:  #往后倒了一定时间后起来
            count1 = 0
            if not control_lock:
                control_lock = True
                AGC.runActionGroup('stand_flip')
                
        elif count2 >= 3:  #后前倒了一定时间后起来
            count2 = 0
            if control_lock:
                control_lock = False
                AGC.runActionGroup('stand_low')

        else:
            time.sleep(0.01)

