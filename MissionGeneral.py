from ProtocolMCU.Application import class_application
from time import sleep
import threading
import cv2
from Logger import logger
from RadarDriver.RadarDriver import LD_Radar
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


class Mission:
    def __init__(self, fc: class_application, camera: img_mapping_system, radar: LD_Radar, pid_tunings):
        '''
        完成基本的实例的定义
        完成底层初始化
        完成PID的初始化(实时控制的四个维度控制PID)
        '''
        self.fc = fc
        self.cam = camera
        self.radar = radar

        self.inital_yaw = self.fc.state.yaw.value
        self.cruise_height = 120

        ########## PID ##########
        self.pid_tunings = pid_tunings  # PID参数 (仅导航XY使用)
        self.current_pos = [0, 0, 0]  # x, y, yaw
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
        # self.paused = False
        # vision-debug()

    def stop(self):
        self.running_flag = False
        self.fc.stop_realtime_control()

    def run(self):
        '''
        仅完成相关的变量的定义'''
        fc = self.fc
        cam = self.cam
        radar = self.radar
        self.running_flag = True

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

    def navigation_to_waypoint(self, waypoint):
        self.pos_x_pid.SetPoint = waypoint[0]
        self.pos_y_pid.SetPoint = waypoint[1]

    def navigation_task(self):
        paused = False
        while self.running:
            sleep(0.01)
            if (
                self.navigation_flag
                and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
                if paused:
                    paused = False
                    self.navi_x_pid.set_auto_mode(True, last_output=0)
                    self.navi_y_pid.set_auto_mode(True, last_output=0)
                    self.navi_yaw_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Navigation resumed")

                out_x = None
                out_y = None
                out_yaw = None
                if current_x > 0:  # 0 为无效值
                    out_x = int(self.navi_x_pid(current_x))
                    if out_x is not None:
                        self.fc.update_realtime_control(vel_x=out_x)
                if current_y > 0:
                    out_y = int(self.navi_y_pid(current_y))
                    if out_y is not None:
                        self.fc.update_realtime_control(vel_y=out_y)
                out_yaw = int(self.navi_yaw_pid(current_yaw))
                if out_yaw is not None:
                    self.fc.update_realtime_control(yaw=out_yaw)
                if False:  # debug
                    logger.debug(
                        f"[MISSION] Current pose: {current_x}, {current_y}, {current_yaw}; Output: {out_x}, {out_y}, {out_yaw}"
                    )
            else:
                self.radar.stop_resolve_pose()
                if not paused:
                    paused = True
                    self.navi_x_pid.set_auto_mode(False)
                    self.navi_y_pid.set_auto_mode(False)
                    self.navi_yaw_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.info("[MISSION] Navigation paused")
