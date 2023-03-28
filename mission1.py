import cv2
from RadarDriver.RadarDriver import LD_Radar
from ProtocolMCU.Application import class_application
from MissionGeneral import Mission


pid_tunnings = {
    "default": (0.35, 0, 0.08),  # 导航
    "delivery": (0.4, 0.05, 0.16),  # 配送
    "landing": (0.4, 0.05, 0.16),  # 降落
}


class Mission1(Mission):
    def __init__(self, fc: class_application, camera: cv2.VideoCapture, radar: LD_Radar, pid_tunings):
        super().__init__(fc, camera, radar, pid_tunings)

    # 对run函数进行重写
    def run(self):
        super().run()
