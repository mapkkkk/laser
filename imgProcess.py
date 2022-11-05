# coding=UTF-8
import cv2 as cv
import numpy as np

'''
图像处理主程序
python储存图片的方式是按照行X列的方式的，例如720p的图就是720X1080
'''

# 霍夫变换参数
rho = 1
theta = np.pi / 90
threshold = 80
min_line_len = 85
max_line_gap = 5


# 摄像头初始化
def init_cap():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_BRIGHTNESS, 130)  # 亮度 130
    fourcc = cv.VideoWriter_fourcc('M', 'J', 'P', 'G')
    cap.set(cv.CAP_PROP_FOURCC, fourcc)
    cap.set(cv.CAP_PROP_FPS, 30)
    cap.open(0)
    return cap


# 视觉函数
def visualOpen(cap):
    a_total_x = 0
    error_y = 0
    # 一定要注意这里的写法，不然会运行不了
    print('opened')
    if cap.isOpened():
        res, frame = cap.read()
        if res:
            # 进到图像处理，返回直线的数据，理想情况下只有一条线
            print('frame')
            image = cv.rotate(frame, cv.ROTATE_180)
            lines = basic_process(image)
            # 画在屏幕上显示看看
            if lines is not None:
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        cv.line(image, (x1, y1), (x2, y2), (0, 255, 0), 3)

            a_total_x, a_total_y, a_total_k = lines_process(lines)
            # 显示一下原始图像
            cv.imshow("Webcam", image)
            key_pressed = cv.waitKey(100)
            error_y = a_total_y - 240
    return error_y, a_total_x


def basic_process(img):
    # 灰度化处理
    img_gray = cv.cvtColor(img, cv.COLOR_BGRA2GRAY)

    # 高斯模糊
    dst = cv.equalizeHist(img_gray)
    gaussian = cv.GaussianBlur(dst, (3, 3), 0)

    # 二值化
    ret, thresh = cv.threshold(gaussian, 15, 255, cv.THRESH_BINARY_INV)

    # 腐蚀膨胀
    kernel = np.ones((3, 3), np.uint8)
    kernel_lg = np.ones((3, 3), np.uint8)
    erode = cv.erode(thresh, kernel, iterations=1)
    dilate = cv.dilate(erode, kernel_lg, iterations=1)

    # 在这里多加以此高斯似乎意义不是很大，但是值得试一试
    dst_final = cv.GaussianBlur(dilate, (3, 3), 0)
    cv.imshow('dilate+blur', dst_final)

    # 找出边缘，似乎必要性不是太大，之后再看
    img_edge = cv.Canny(dst_final, 70, 150)
    cv.imshow('edge', img_edge)

    # 霍夫直线变换
    lines = cv.HoughLinesP(img_edge, rho, theta, threshold, min_line_len, max_line_gap)

    # 返回直线数据，之后再处理
    return lines


# 直线数据处理
def lines_process(lines):
    total_x = 0
    total_y = 0
    total_k = 0
    total_lines_x = 1
    total_lines_y = 1

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                k = 10
                if x2 - x1 is not 0:
                    k = (y2 - y1) / (x2 - x1)
                if 0.3 > k > -0.3:
                    total_y += (y1 + y2) / 2
                    total_k += (y2 - y1) / (x2 - x1)
                    total_lines_y += 1
                elif k > 5 or k < -5:
                    total_x += (x1 + x2) / 2
                    total_lines_x += 1

    a_total_x = total_x / total_lines_x
    a_total_y = total_y / total_lines_y
    a_total_k = total_k / total_lines_y

    return a_total_x, a_total_y, a_total_k
