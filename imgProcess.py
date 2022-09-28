import cv2
import numpy as np



def imgProcess(img):
    img_blur = cv2.blur(img,(5,5))
    img_gray = cv2.cvtColor(img_blur,cv2.COLOR_BGRA2GRAY)
    retval, img_dst = cv2.threshold(img_gray, 0, 255, cv2.THRESH_OTSU)
    img_exdst = cv2.dilate(img_dst, None, 2)
    img_edge = cv2.Canny(img_exdst,50,100)
    rows, cols = img_edge.shape
    points = np.array([[(500, 0), (600, rows), (600, 0), (500, rows)]])
    roi_edges = roi_mask(img_edge, points)
    return roi_edges


def roi_mask(img, corner_points):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, corner_points, 255)
    masked_img = cv2.bitwise_and(img, mask)
    return masked_img