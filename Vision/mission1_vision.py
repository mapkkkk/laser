import cv2 as cv
import ImgProcess
from ImgProcess import (
    find_QRcode_zbar,
    change_cam_resolution,
    get_ROI,

)
from Logger import logger
from time import sleep
import threading


class img_mapping_system:
    def __init__(self, cam: cv.VideoCapture) -> None:
        self.cam = cam
        self.process = ImgProcess
        self.process_number = 0
        self.running = False
        self.thread_list = []
        self.cam_pos = [0, 0, 0]    # 相机的位置(其实就是无人机的相对位置,考虑加上一个偏置)

    def visual_navi_task(self):
        cam = self.cam
        change_cam_resolution(cam, 800, 600)
        while (True):
            sleep(0.01)
            img = cam.read()
            if (self.process_number == 0):
                pass
            else:
                img = self.preprocess(img=img, number=self.process_number)

    def find_lines(self, img):
        pass

    def preprocess(self, img, number):
        if number == 1:
            pass
        elif number == 2:
            pass
        elif number == 3:
            pass

    def postprocess(self, img):
        pass

    def start(self):
        """
        开始视觉线程,持续读入图片,但仅在有必要是进行处理
        """
        if self.running == False:
            self.running == True
            thread = threading.Thread(
                target=self.visual_navi_task, daemon=True)
            thread.start()
            self.thread_list.append(thread)
            logger.info("[visual] img rev_process start")
