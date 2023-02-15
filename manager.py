# coding=UTF-8
import time
import cv2 as cv
from time import sleep

from Base import base_communicate
from Protocol import class_protocol
from PID import PID
from ImgProcess import init_cap
from ImgProcess import visualOpen

'''
目前写的这一部分只是用于任务调度，可能以后就是用来执行任务用的主程序大概
注意，绝对不要让实例的名字和类的一样，不然会变得不幸
注意，在树莓派上一定要将串口改过来，两者性能差距太大了
其他组都没改是因为它们都没有使用实时控制帧做第一题
HZW
'''
# 定义串口
# 这个串口是树莓派的串口TX&RX
#
port = '/dev/ttyAMA0'

# 实例化任务控制

# 开启摄像头
cam = cv.VideoCapture(0)
if not cam.isOpened():
    cam.open(0)
assert cam.isOpened()

# 初始化PID
P = 1
I = 0.2
D = 0.002
pidx = PID(P, I, D)
# 50ms更新PID
pidx.setSampleTime(0.1)

# 视觉测试代码
# while 1:
#     error_y, average_x = visualOpen(cap)
#     error_y = error_y/20
#     pidx.update(error_y)
#     control_y = pidx.output
#     print(control_y)


# PID更新周期设为10ms

state = 0
time_count = 0
current_time = 0
last_time = 0
