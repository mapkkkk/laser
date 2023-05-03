# coding=UTF-8
import cv2 as cv
import numpy as np
import os
import time
from typing import List, Optional, Tuple, Union, Any
# from pyzbar import pyzbar
'''
python储存图片的方式是按照行X列的方式的,例如720p的图就是720X1080
'''

# 霍夫变换参数
rho = 1
theta = np.pi / 90
threshold = 80
min_line_len = 85
max_line_gap = 5

_DEBUG = False


def get_ROI(
    img,
    ROI: Tuple[Union[int, float]],
) -> np.ndarray:
    """
    获取兴趣区
    ROI: 若<=1则视为相对图像尺寸的比例值
    """
    x, y, w, h = ROI
    if (x + y + w + h) <= 4:
        x = int(x * img.shape[1])
        y = int(y * img.shape[0])
        w = int(w * img.shape[1])
        h = int(h * img.shape[0])
    return img[y: y + h, x: x + w]


def rotate_img(image, angle, fill_color=(0, 0, 0)):
    """
    任意角度旋转图片
    angle: 旋转角度，顺时针方向, 角度制
    fill_color: 填充颜色
    """
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)
    M = cv.getRotationMatrix2D((cX, cY), -angle, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY
    return cv.warpAffine(image, M, (nW, nH), borderValue=fill_color)


def set_cam_autowb(cam, enable=True, manual_temp=5500):
    """
    设置摄像头自动白平衡
    enable: 是否启用自动白平衡
    manual_temp: 手动模式下的色温
    """
    cam.set(cv.CAP_PROP_AUTO_WB, int(enable))
    if not enable:
        cam.set(cv.CAP_PROP_WB_TEMPERATURE, manual_temp)


def set_cam_autoexp(cam, enable=True, manual_exposure=0.25):
    """
    设置摄像头自动曝光
    enable: 是否启用自动曝光
    manual_exposure: 手动模式下的曝光时间
    """
    cam.set(cv.CAP_PROP_AUTO_EXPOSURE, int(enable))
    if not enable:
        cam.set(cv.CAP_PROP_EXPOSURE, manual_exposure)


class HSV(object):
    """
    常用色值HSV边界
    """

    RED_UPPER = np.array([10, 255, 255])
    RED_LOWER = np.array([0, 43, 46])
    RED_UPPER2 = np.array([180, 255, 255])
    RED_LOWER2 = np.array([156, 43, 46])
    YELLOW_UPPER = np.array([34, 255, 255])
    YELLOW_LOWER = np.array([26, 43, 46])
    GREEN_UPPER = np.array([77, 255, 255])
    GREEN_LOWER = np.array([35, 43, 46])
    BLUE_UPPER = np.array([124, 255, 255])
    BLUE_LOWER = np.array([100, 43, 46])
    ORANGE_UPPER = np.array([25, 255, 255])
    ORANGE_LOWER = np.array([11, 43, 46])
    CYAN_UPPER = np.array([99, 255, 255])
    CYAN_LOWER = np.array([78, 43, 46])
    PURPLE_UPPER = np.array([155, 255, 255])
    PURPLE_LOWER = np.array([125, 43, 46])
    BLACK_UPPER = np.array([180, 255, 46])
    BLACK_LOWER = np.array([0, 0, 0])
    GRAY_UPPER = np.array([180, 43, 220])
    GRAY_LOWER = np.array([0, 0, 46])
    WHITE_UPPER = np.array([180, 30, 255])
    WHITE_LOWER = np.array([0, 0, 221])


def change_cam_resolution(cam, width: int, height: int, fps: int = 60):
    """
    改变摄像头分辨率
    return 切换后的 宽,高,fps
    """
    cam.set(cv.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    cam.set(cv.CAP_PROP_FPS, fps)
    return (
        cam.get(cv.CAP_PROP_FRAME_WIDTH),
        cam.get(cv.CAP_PROP_FRAME_HEIGHT),
        cam.get(cv.CAP_PROP_FPS),
    )


def color_recognition(img, threshold=0.4) -> Union[str, None]:
    """
    颜色识别(红绿蓝黄)
    threshold: 颜色占比阈值, 大于该阈值则认为是该颜色
    return: 识别结果的文本, 无法识别则为None
    """
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    maske_r = cv.bitwise_or(
        cv.inRange(hsv, HSV.RED_LOWER, HSV.RED_UPPER),
        cv.inRange(hsv, HSV.RED_LOWER2, HSV.RED_UPPER2),
    )
    mask_g = cv.inRange(hsv, HSV.GREEN_LOWER, HSV.GREEN_UPPER)
    mask_b = cv.inRange(hsv, HSV.BLUE_LOWER, HSV.BLUE_UPPER)
    red_count = cv.countNonZero(maske_r)
    green_count = cv.countNonZero(mask_g)
    blue_count = cv.countNonZero(mask_b)
    max_count = max(red_count, green_count, blue_count)
    tho = int(img.shape[0] * img.shape[1] * threshold)
    if max_count == red_count and red_count > tho:
        return "red"
    elif max_count == green_count and green_count > tho:
        return "green"
    elif max_count == blue_count and blue_count > tho:
        return "blue"
    else:
        return None


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
    lines = cv.HoughLinesP(img_edge, rho, theta,
                           threshold, min_line_len, max_line_gap)

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
                if x2 - x1 != 0:
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


# 神经网络
# sigmoid函数
def sigmoid(x):
    return 1.0 / (1 + np.exp(-x))


# tanh函数
def tanh(x):
    return 2.0 / (1 + np.exp(-2 * x)) - 1


def draw_pred(frame, class_name, conf, left, top, right, bottom):
    """
    绘制预测结果
    """
    cv.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)
    label = f"{class_name}: {conf:.2f}"
    label_size, _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    top = max(top - 10, label_size[1])
    left = max(left, 0)
    cv.putText(
        frame,
        label,
        (left, top),
        cv.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        thickness=2,
    )


class FastestDetOnnx:
    def __init__(self, confThreshold=0.6, nmsThreshold=0.45, drawOutput=False):
        """
        FastestDet 目标检测网络
        confThreshold: 置信度阈值
        nmsThreshold: 非极大值抑制阈值
        """
        import onnxruntime

        path = os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "models")
        path_names = os.path.join(path, "FastestDet.names")  # 识别类别
        path_onnx = os.path.join(path, "FastestDet.onnx")
        self.classes = list(
            map(lambda x: x.strip(), open(path_names, "r").readlines()))
        self.inpWidth = 500
        self.inpHeight = 500
        self.session = onnxruntime.InferenceSession(path_onnx)
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.drawOutput = drawOutput

    def preprocess(self, src_img, size):
        """
        前处理, 对输入图像进行归一化
        """
        output = cv.resize(
            src_img, (size[0], size[1]), interpolation=cv.INTER_AREA)
        output = output.transpose(2, 0, 1)
        output = output.reshape((1, 3, size[1], size[0]))

        return output.astype("float32")

    def postprocess(self, frame, outs):
        """
        后处理, 对输出进行筛选
        """
        outs = outs.transpose(1, 2, 0)
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        feature_height = outs.shape[0]
        feature_width = outs.shape[1]
        preds = []
        confidences = []
        boxes = []
        ret = []
        for h in range(feature_height):
            for w in range(feature_width):
                data = outs[h][w]
                obj_score, cls_score = data[0], data[5:].max()
                score = (obj_score ** 0.6) * (cls_score ** 0.4)
                if score > self.confThreshold:
                    classId = np.argmax(data[5:])
                    # 检测框中心点偏移
                    x_offset, y_offset = tanh(data[1]), tanh(data[2])
                    # 检测框归一化后的宽高
                    box_width, box_height = sigmoid(data[3]), sigmoid(data[4])
                    # 检测框归一化后中心点
                    box_cx = (w + x_offset) / feature_width
                    box_cy = (h + y_offset) / feature_height
                    x1, y1 = (box_cx - 0.5 * box_width,
                              box_cy - 0.5 * box_height)
                    x2, y2 = (box_cx + 0.5 * box_width,
                              box_cy + 0.5 * box_height)
                    x1, y1, x2, y2 = (
                        int(x1 * frameWidth),
                        int(y1 * frameHeight),
                        int(x2 * frameWidth),
                        int(y2 * frameHeight),
                    )
                    preds.append([x1, y1, x2, y2, score, classId])
                    boxes.append([x1, y1, x2 - x1, y2 - y1])
                    confidences.append(score)
        indices = cv.dnn.NMSBoxes(
            boxes, confidences, self.confThreshold, self.nmsThreshold
        )
        indices = np.array(indices).flatten().tolist()
        for i in indices:
            pred = preds[i]
            score, classId = pred[4], int(pred[5])
            x1, y1, x2, y2 = pred[0], pred[1], pred[2], pred[3]
            center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
            ret.append(((center_x, center_y), self.classes[classId], score))
            if self.drawOutput:
                draw_pred(frame, self.classes[classId], score, x1, y1, x2, y2)
        return ret

    def detect(self, frame):
        """
        执行识别
        return: 识别结果列表: (中点坐标, 类型名称, 置信度)
        """
        data = self.__preprocess(frame, (self.inpWidth, self.inpHeight))
        input_name = self.session.get_inputs()[0].name
        feature_map = self.session.run([], {input_name: data})[0][0]
        return self.__postprocess(frame, feature_map)


def onnxruntime_init():
    if __name__ == "__main__":
        # 读取图片
        cam = cv.VideoCapture(0)
        # 模型输入的宽高
        # input_width, input_height = 500, 500
        # 加载模型
        fd = FastestDetOnnx(drawOutput=False)
        # 目标检测
        while True:
            img = cam.read()[1]
            start = time.perf_counter()
            # ret = fd.detect(img)
            end = time.perf_counter()
            time_ = (end - start) * 1000.0
            print("forward time:%fms" % time_)
            cv.imshow("img", img)
            if cv.waitKey(1) == 27:
                break
        return cam, fd


def onnxruntime(cam, fd):
    img = cam.read()[1]
    start = time.perf_counter()
    ret = fd.detect(img)
    end = time.perf_counter()
    time_ = (end - start) * 1000.0
    print("forward time:%fms" % time_)
    cv.imshow("img", img)


def black_line(
    image, type: int = 1, theta_threshold=0.25
) -> Tuple[bool, float, float, float]:
    """
    寻找画面中的黑线并返回数据
    type: 0:横线 1:竖线
    theta_threshold: 角度容许误差(不能超过45度)
    return: 是否查找到黑线, x偏移值(右正), y偏移值(下正), 弧度偏移值(顺时针正)
    """
    ######### 参数设置 #########
    LOWER = np.array([0, 60, 0])
    UPPER = np.array([150, 255, 75])
    HOUGH_THRESHOLD = 200
    ###########################
    hsv_img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv_img, LOWER, UPPER)
    if _DEBUG:
        cv.imshow("Process", mask)

    target_theta = 0 if type == 1 else np.pi / 2
    # lines = cv2.HoughLines(mask, 1, np.pi/180, threshold=400, max_theta=0.1)
    if type == 0:  # 横线
        lines = cv.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            min_theta=target_theta - theta_threshold,
            max_theta=target_theta + theta_threshold,
        )
    else:  # 竖线
        lines = cv.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            max_theta=theta_threshold,
        )
        lines2 = cv.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            min_theta=np.pi - theta_threshold,
        )
        if lines is not None and lines2 is not None:
            lines = np.concatenate((lines, lines2))
        elif lines is None and lines2 is not None:
            lines = lines2

    if lines is not None:
        for line in lines:
            r, theta = line[0]
            x0 = r * np.cos(theta)
            y0 = r * np.sin(theta)
            x1 = int(x0 - 1000 * np.sin(theta))
            y1 = int(y0 + 1000 * np.cos(theta))
            x2 = int(x0 + 1000 * np.sin(theta))
            y2 = int(y0 - 1000 * np.cos(theta))
            if _DEBUG:
                cv.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv.imshow("Result", image)
            x = abs((x1 + x2) / 2)
            y = abs((y1 + y2) / 2)
            size = image.shape
            x_offset = x - size[1] / 2
            y_offset = y - size[0] / 2
            if theta > np.pi / 2 and type == 1:
                t_offset = theta - target_theta - np.pi
            else:
                t_offset = theta - target_theta
            return True, x_offset, y_offset, t_offset
    return False, 0, 0, 0


def find_QRcode_zbar(frame) -> Union[
    tuple[bool, Union[Union[int, float], Any], Union[Union[int, float], Any], Any], tuple[bool, int, int, str]]:
    """
    使用pyzbar寻找条码
    return: 是否找到条码, x偏移值(右正), y偏移值(下正), 条码内容
    """
    image = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # 转换成灰度图
    barcodes = pyzbar.decode(image)
    if barcodes != []:
        size = image.shape
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            data = barcode.data.decode("utf-8")
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            x_offset = cx - size[1] / 2
            y_offset = cy - size[0] / 2
            if _DEBUG:
                image = frame.copy()
                cv.circle(image, (cx, cy), 2, (0, 255, 0), 8)
                cv.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv.putText(
                    image,
                    data,
                    (x, y - 20),
                    cv.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )
                cv.imshow("Result", image)
            return True, x_offset, y_offset, data
    return False, 0, 0, ""


class KCF_Tracker:  # OPENCV KCF Tracker
    def __init__(self) -> None:
        self.frame = None
        self.selection = None
        self.drag_start = None
        self.track_window = None
        self.track_start = False
        cv.namedWindow("KCFTracker", cv.WINDOW_KEEPRATIO |
                       cv.WINDOW_AUTOSIZE)
        cv.setMouseCallback("KCFTracker", self.onmouse)

    def onmouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.track_start = False
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv.EVENT_LBUTTONUP:
            self.drag_start = None
            self.selection = None
            self.track_window = (xmin, ymin, xmax - xmin, ymax - ymin)
            if (
                self.track_window
                and self.track_window[2] > 0
                and self.track_window[3] > 0
            ):
                self.track_start = True
                self.tracker = cv.TrackerKCF_create()
                self.tracker.init(self.frame, self.track_window)
            else:
                self.track_start = False
                self.track_window = None
                self.tracker = None

    def process(self, frame):
        self.frame = frame.copy()
        if self.selection:
            x0, y0, x1, y1 = self.selection
            cv.rectangle(self.frame, (x0, y0), (x1, y1), (255, 0, 0), 2, 1)
        self.track_ok = None
        if self.track_start:
            self.track_ok, bbox = self.tracker.update(frame)
        if self.track_ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv.rectangle(self.frame, p1, p2, (255, 0, 0), 2, 1)
        elif not self.track_start:
            cv.putText(
                self.frame,
                "No tracking target selected",
                (0, 20),
                cv.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )
        elif not self.track_ok:
            cv.putText(
                self.frame,
                "Tracking failed",
                (0, 20),
                cv.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )
        cv.imshow("KCFTracker", self.frame)
