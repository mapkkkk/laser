import threading
import cv2 as cv
from time import time
from ProtocolMCU.Application import class_application
from time import sleep
from others.Logger import logger
from RadarDrivers.RadarDriver import LD_Radar
from simple_pid import PID
from Vision.mission1_vision import img_mapping_system
"""
通用任务模版：
包含：
1. 高度自稳
2. 自动前往坐标点(前提是雷达已经完成定位)
3. 定点起飞/降落
若要使用,直接集成,并且重写run函数
由于在这个项目中我们没有拿到雷达,所以定点和导航的所有代码得重写
"""


class Mission_General:
    def __init__(self, fc: class_application, camera: cv.VideoCapture, radar: LD_Radar):
        '''
        完成基本的实例的定义
        完成底层初始化
        完成PID的初始化(实时控制的四个维度控制PID)
        '''
        self.fc = fc
        self.cam = camera
        self.radar = radar
        self.DEBUG = False
        self.inital_yaw = self.fc.state.yaw.value
        self.cruise_height = 120

        ########## PID ##########
        self.pid_tunings = {
            "default": (0.35, 0, 0.08),  # 导航
            "delivery": (0.4, 0.05, 0.16),  # 配送
            "landing": (0.4, 0.05, 0.16),  # 降落
        }  # PID参数 (仅导航XY使用) default settings
        self.height_pid = PID(
            0.8,
            0.0,
            0.1,
            setpoint=0,
            output_limits=(-30, 30),
            auto_mode=False,
        )
        self.height_pid.SetPoint = 140  # 定高140cm
        self.pos_x_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.pos_y_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.yaw_pid = PID(
            0.2,
            0.0,
            0.0,
            setpoint=0,
            output_limits=(-45, 45),
            auto_mode=False,
        )

        ######### FLAGS #########
        self.keep_height_flag = False   # 定高flag
        self.running_flag = False   # 运行flag
        self.thread_list = []    # 线程列表
        self.navigation_flag = False
        self.navigation_speed = 35  # 导航速度
        self.precision_speed = 25  # 精确速度
        # self.paused = False
        # vision-debug()

    def stop(self):
        self.running_flag = False
        self.fc.stop_realtime_control()

    def switch_pid(self, pid):
        """
        切换PID参数
        """
        tuning = self.pid_tunings.get(pid, self.pid_tunings["default"])
        self.pos_x_pid.tunings = tuning
        self.pos_y_pid.tunings = tuning
        logger.info(f"[MISSION] PID Tunings set to {pid}: {tuning}")

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
                    self.height_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Keep Height resumed")
                out_height = int(self.height_pid(self.fc.state.alt_add.value))
                self.fc.update_realtime_control(vel_z=out_height)
            else:
                if not paused:
                    paused = True
                    self.height_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_z=0)
                    logger.info("[MISSION] Keep height paused")

    def point_takeoff(self, point):
        """
        定点起飞
        """
        logger.info(f"[MISSION] Takeoff at {point}")
        self.navigation_flag = False
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        self.fc.unlock()
        inital_yaw = self.fc.state.yaw.value
        sleep(2)  # 等待电机启动
        self.fc.takeoff(140)
        self.fc.wait_for_takeoff_done()
        self.fc.set_yaw(inital_yaw, 25)
        self.fc.wait_for_hovering(2)
        # 闭环定高
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        self.height_pid.SetPoint = self.cruise_height
        self.keep_height_flag = True
        sleep(2)
        self.navigation_to_waypoint(point)  # 设置路径点(认为是当前位置)
        self.switch_pid("default")
        sleep(0.1)
        self.navigation_flag = True
        self.set_navigation_speed(self.navigation_speed)

    def pointing_landing(self, point):
        """
        定点降落
        """
        logger.info(f"[MISSION] Landing at {point}")
        self.navigation_to_waypoint(point)
        self.wait_for_waypoint()
        self.set_navigation_speed(self.precision_speed)
        self.switch_pid("landing")
        sleep(1)
        self.height_pid.setpoint = 60
        sleep(1.5)
        self.height_pid.setpoint = 30
        sleep(1.5)
        self.height_pid.setpoint = 20
        sleep(2)
        self.wait_for_waypoint()
        # self.height_pid.setpoint = 0
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        sleep(0.1)
        self.fc.land()
        self.fc.wait_for_lock(5)
        self.fc.lock()

    def navigation_to_waypoint(self, waypoint):
        self.pos_x_pid.SetPoint = waypoint[0]
        self.pos_y_pid.SetPoint = waypoint[1]

    def set_navigation_speed(self, speed):
        speed = abs(speed)
        self.pos_x_pid.output_limits = (-speed, speed)
        self.pos_y_pid.output_limits = (-speed, speed)

    def wait_for_waypoint(self, time_thres=1, pos_thres=15, timeout=30):
        time_count = 0
        time_start = time()
        while True:
            sleep(0.1)
            if self.reached_waypoint(pos_thres):
                time_count += 0.1
            if time_count >= time_thres:
                logger.info("[MISSION] Reached waypoint")
                return
            if time() - time_start > timeout:
                logger.warning("[MISSION] Waypoint overtime")
                return

    def reached_waypoint(self, pos_thres=15):
        return (
            abs(self.radar.rt_pose[0] - self.pos_x_pid.setpoint) < pos_thres
            and abs(self.radar.rt_pose[1] - self.pos_y_pid.setpoint) < pos_thres
        )

    def navigation_task_radar(self):
        """
        这里是雷达的参数作为当前位置
        然后呢我们也有目标位置
        所以就有PID
        所以就能往那边走
        """
        ######## 解算参数 ########
        SIZE = 1000
        SCALE_RATIO = 0.5
        LOW_PASS_RATIO = 0.6
        ########################
        paused = False
        while self.running_flag:
            sleep(0.01)
            if (
                self.navigation_flag
                and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
                if paused:
                    paused = False
                    self.pos_x_pid.set_auto_mode(True, last_output=0)
                    self.pos_y_pid.set_auto_mode(True, last_output=0)
                    self.yaw_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Navigation resumed")
                if not self.radar._rtpose_flag:
                    self.radar.start_resolve_pose(
                        size=SIZE,
                        scale_ratio=SCALE_RATIO,
                        low_pass_ratio=LOW_PASS_RATIO,
                    )
                    logger.info("[MISSION] Resolve pose started")
                    sleep(0.01)
                if self.radar.rt_pose_update_event.wait(1):  # 等待地图更新,这个event是给这个线程用的
                    self.radar.rt_pose_update_event.clear()     # 地图更新，重新上锁，等待再次set
                    current_x = self.radar.rt_pose[0]
                    current_y = self.radar.rt_pose[1]
                    # if current_x > 0 and current_y > 0:
                    #     self.fc.send_general_position(x=current_x, y=current_y)
                    current_yaw = self.radar.rt_pose[2]
                    out_x = None
                    out_y = None
                    out_yaw = None
                    if current_x > 0:  # 0 为无效值
                        out_x = int(self.pos_x_pid(current_x))
                        if out_x is not None:
                            self.fc.update_realtime_control(vel_x=out_x)
                    if current_y > 0:
                        out_y = int(self.pos_y_pid(current_y))
                        if out_y is not None:
                            self.fc.update_realtime_control(vel_y=out_y)
                    out_yaw = int(self.yaw_pid(current_yaw))
                    if out_yaw is not None:
                        self.fc.update_realtime_control(yaw=out_yaw)
                    if self.DEBUG:  # debug
                        logger.debug(
                            f"[MISSION] Current pose: {current_x}, {current_y}, {current_yaw}; Output: {out_x}, {out_y}, {out_yaw}"
                        )
            else:
                self.radar.stop_resolve_pose()
                if not paused:
                    paused = True
                    self.pos_x_pid.set_auto_mode(False)
                    self.pos_y_pid.set_auto_mode(False)
                    self.yaw_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.info("[MISSION] Navigation paused")
