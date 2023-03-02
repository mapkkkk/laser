# coding=UTF-8
import cv2 as cv
import numpy as np
import os
import time

'''
图像处理主程序
python储存图片的方式是按照行X列的方式的,例如720p的图就是720X1080
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
            # key_pressed = cv.waitKey(100)
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
    labelSize, _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    top = max(top - 10, labelSize[1])
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

    def __preprocess(self, src_img, size):
        """
        前处理, 对输入图像进行归一化
        """
        output = cv.resize(
            src_img, (size[0], size[1]), interpolation=cv.INTER_AREA)
        output = output.transpose(2, 0, 1)
        output = output.reshape((1, 3, size[1], size[0]))

        return output.astype("float32")

    def __postprocess(self, frame, outs):
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
    # ret = fd.detect(img)
    end = time.perf_counter()
    time_ = (end - start) * 1000.0
    print("forward time:%fms" % time_)
    cv.imshow("img", img)
