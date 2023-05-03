import cv2
import threading
from RadarDrivers.RadarDriver import LD_Radar
from ProtocolMCU.Application import class_application
from MissionGeneralRadar import Mission_General
from others.Logger import logger
from time import sleep
from Vision.ImgProcess import (
    change_cam_resolution,
    set_cam_autowb,
)


class Mission(Mission_General):
    def __init__(self, fc: class_application, camera: cv2.VideoCapture, radar: LD_Radar, pid_tunings):
        super().__init__(fc, camera, radar)
        self.pid_tunings = {
            "default": (0.35, 0, 0.08),  # 导航
            "delivery": (0.4, 0.05, 0.16),  # 配送
            "landing": (0.4, 0.05, 0.16),  # 降落
        }  # PID参数 (仅导航XY使用)

    # 建立run函数
    def run(self):
        fc = self.fc
        cam = self.cam
        radar = self.radar
        ########### 参数#########
        self.running_flag = True
        self.camera_down_pwm = 32.5
        self.camera_up_pwm = 72
        self.navigation_speed = 35  # 导航速度
        self.precision_speed = 25  # 精确速度
        self.cruise_height = 140  # 巡航高度
        self.goods_height = 80  # 处理物品高度
        ################ 启动线程 ################
        self.running = True
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task, daemon=True)
        )
        self.thread_list[-1].start()
        self.thread_list.append(
            threading.Thread(target=self.navigation_task_radar(), daemon=True)
        )
        self.thread_list[-1].start()
        logger.info("[MISSION] Threads started")
        ################ 初始化 ################
        change_cam_resolution(cam, 800, 600)
        set_cam_autowb(cam, False)  # 关闭自动白平衡
        for _ in range(10):
            cam.read()
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.set_navigation_speed(self.navigation_speed)
        self.fc.start_realtime_control(20)  # 频率20Hz
        self.switch_pid("default")
        BASE_POINT = [self.radar.rt_pose[0], self.radar.rt_pose[1]]
        LANDING_POINT = BASE_POINT
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
        self.point_takeoff(BASE_POINT)
        ################ 开始任务 ################
        # 开始按照点一个一个前往
        for target_point in target_points:
            x, y = target_point
            target_point_pos = POINTS_ARR[y, x]
            self.navigation_to_waypoint(target_point_pos)
            self.wait_for_waypoint()
        # 回到基地点
        logger.info("[MISSION] Go to base")
        self.navigation_to_waypoint(BASE_POINT)
        self.wait_for_waypoint()
        self.pointing_landing(LANDING_POINT)
        logger.info("[MISSION] Misson-1 Finished")
