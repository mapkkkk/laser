import sys
import syslog
import time
import serial
import cv2 as cv
import numpy as np
import imgProcess
import lineTracker
from dataProcess import FC_Base_Uart_Communication
from Base import Byte_Var

# opencv初始化
cap = cv.VideoCapture(0)
cap.open(0)

# 霍夫变换参数
rho = 1
theta = np.pi / 180
threshold = 15
min_line_len = 40
max_line_gap = 20

# 定义串口
port = ...
# 实例化发送类
sendclass = FC_Base_Uart_Communication(port)


class data_send_temp_struct:
    takeofforlanding = Byte_Var("u8", int, name="takeofforlanding")
    vel_x = Byte_Var("s16", int, name="vel_x")  # cm/s
    vel_y = Byte_Var("s16", int, name="vel_y")  # cm/s
    vel_z = Byte_Var("s16", int, name="vel_z")  # cm/s
    pos_x = Byte_Var("s32", int, name="pos_x")  # cm
    pos_y = Byte_Var("s32", int, name="pos_y")  # cm
    testint = Byte_Var("u8", int, name="testint")
    testint2 = Byte_Var("u8", int, name="testint2")
    testfloat = Byte_Var("u16", float, name="testfloat")
    testbool = Byte_Var("u8", bool, name="testbool")

    def __init__(self):
        self.testint.value(6)
        self.testint.value(12)
        self.testfloat.value(6.6)
        self.testbool.value(True)

    def dataoutput(self, option: str):
        if option == 'vel_x':
            return self.vel_x
        elif option == 'vel_y':
            return self.vel_y
        elif option == 'vel_z':
            return self.vel_z
        elif option == 'testint':
            return self.testint
        elif option == 'testint2':
            return self.testint2
        elif option == 'testfloat':
            return self.testfloat
        elif option == 'testbool':
            return self.testbool


def datasendtest():
    sendclass.send_data_to_fc(data_send_temp_struct.testint.bytes, option=0x22)
    

def visualOpen() -> None:
    # 一定要注意这里的写法，不然会运行不了
    while cap.isOpened():
        res, frame = cap.read()
        if res:
            cv.imshow("Webcam", frame)
            # 对图像的二值化处理
            result = imgProcess.imgProcess(frame)
            cv.imshow('result', result)
            # 找出线，并且计算运动方式
            drawing, lines = lineTracker.hough_lines(result, rho, theta, threshold, min_line_len, max_line_gap)
            cv.imshow('drawing', drawing)
            key_pressed = cv.waitKey(100)
            # print('单机窗口，输入按键，电脑按键为', key_pressed, '按esc键结束')
            if key_pressed == 27:
                break
    cv.destroyAllWindows()
