from Protocol import class_protocol
from PID import PID
from time import sleep
import threading
import cv2


class Mission:
    def __init__(self, fc: class_protocol, camera: cv2.VideoCapture):
        self.fc = fc
        self.cam = camera
        self.inital_yaw = self.fc.state.yaw.value

        ########## PID##########
        self.height_pid = PID(1.2, 0.5, 0.001)
        self.height_pid.SetPoint = 140  # 定高140
        self.pos_y_pid = PID(1.2, 0.5, 0.001)
        self.pos_x_pid = PID(1.2, 0.5, 0.001)

        ######### FLAGS#########
        self.keep_height_flag = False
        self.running_flag = False
        self.thread_list = []
        # vision-debug()

    def stop(self):
        self.running_flag = False
        self.fc.realtime_control_reset()

    def run(self):
        fc = self.fc
        cam = self.cam
        ########## init_value###########
        self.running_flag = True
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task,
                             daemon=True)  # 在添加到列表的时候就已经完成定义了
        )

    def keep_height_task(self):
        paused = False
        while self.running_flag:
            sleep(0.1)
            if (
                    self.keep_height_flag
                    and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
                if paused:
                    paused = False
                    self.height_pid.reset()
                self.height_pid.update(self.fc.state.alt_fused)
                out_vel_z = int(self.height_pid.output)
                self.fc.realtime_control_config(vel_z=out_vel_z)
