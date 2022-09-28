import sys
import syslog
import time
import cv2 as cv
import numpy as np
import imgProcess
import lineTracker

#初始化
cap = cv.VideoCapture(0)
cap.open(0)

#霍夫变换参数
rho = 1
theta = np.pi / 180
threshold = 15
min_line_len = 40
max_line_gap = 20

#一定要注意这里的写法，不然会运行不了
while cap.isOpened():
    res, frame = cap.read()
    if res:
        cv.imshow("Webcam", frame)
        #对图像的二值化处理
        result = imgProcess.imgProcess(frame)
        cv.imshow('result', result)
        #找出线，并且计算运动方式
        drawing, lines = lineTracker.hough_lines(result, rho, theta, threshold, min_line_len, max_line_gap)
        cv.imshow('drawing',drawing)
        key_pressed = cv.waitKey(100)
        #print('单机窗口，输入按键，电脑按键为', key_pressed, '按esc键结束')
        if key_pressed == 27:
            break
cv.destroyAllWindows()



