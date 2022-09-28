import cv2 as cv
import numpy as np


# python储存图片的方式是按照行X列的方式的，例如720p的图就是720X1080


# 霍夫变换（都没理解原理，不过也不管了）
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    # 统计概率霍夫直线变换
    lines = cv.HoughLinesP(img, rho, theta, threshold, min_line_len, max_line_gap)
    # 新建一副空白画布
    drawing = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
    if lines is not None:
        draw_lines(drawing, lines)  # 画出直线检测结果
    return drawing, lines


def draw_lines(img, lines, color=[0, 0, 255], thickness=1):
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv.line(img, (x1, y1), (x2, y2), color, thickness)
