#!/usr/bin/python3
# coding=utf8
import sys
import import_path
import time
import yaml_handle
import HiwonderSDK.TTS as TTS
import HiwonderSDK.ASR as ASR
import kinematics as kinematics
import HiwonderSDK.Board as Board

# 语音控制

HWSONAR = None
ik = kinematics.IK()

servo_data = None
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

try:
    asr = ASR.ASR()
    tts = TTS.TTS()

    debug = True

    if debug:
        asr.eraseWords()
        asr.setMode(2)
        asr.addWords(1, 'kai shi')
        asr.addWords(2, 'wang qian zou')
        asr.addWords(2, 'qian jin')
        asr.addWords(2, 'zhi zou')
        asr.addWords(3, 'wang hou tui')
        asr.addWords(4, 'xiang zuo yi dong')
        asr.addWords(5, 'xiang you yi dong')

    data = asr.getResult()
    Board.setPWMServoPulse(1, 1500, 500)
    Board.setPWMServoPulse(2, servo_data['servo2'], 500)
    ik.stand(ik.initial_pos)
    action_finish = True
    tts.TTSModuleSpeak('[h0][v10][m3]', '准备就绪')
    print('''口令：开始
指令2：往前走
指令2：前进
指令2：直走
指令3：往后退
指令4：向左移动
指令5：向右移动\n''')
    time.sleep(2)
except:
    print('传感器初始化出错')

while True:
    data = asr.getResult()
    if data:
        print('result:', data)
        tts.TTSModuleSpeak('', '收到')
        time.sleep(1)
        if data == 2:
            ik.go_forward(ik.initial_pos, 2, 100, 80, 2)
        elif data == 3:
            ik.back(ik.initial_pos, 2, 100, 80, 2)
        elif data == 4:
            ik.left_move(ik.initial_pos, 2, 100, 80, 2)
        elif data == 5:
            ik.right_move(ik.initial_pos, 2, 100, 80, 2)
    time.sleep(0.01)