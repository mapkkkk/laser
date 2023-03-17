from ProtocolMCU.Application import class_application
from time import sleep
import threading
import cv2
from Logger import logger
from RadarDriver.RadarDriver import LD_Radar
from simple_pid import PID


class Mission:
    def __init__(self, fc: class_application, camera: cv2.VideoCapture, radar: LD_Radar):
        self.fc = fc
        self.cam = camera
        self.radar = radar

        self.inital_yaw = self.fc.state.yaw.value
        self.cruise_height = 120
        self.pid_tunings = {
            "default": (0.35, 0, 0.08),  # 导航
            "delivery": (0.4, 0.05, 0.16),  # 配送
            "landing": (0.4, 0.05, 0.16),  # 降落
        }  # PID参数 (仅导航XY使用)

        ########## PID ##########
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
        self.keep_height_flag = False
        self.running_flag = False
        self.thread_list = []
        self.running = False
        # vision-debug()

    def stop(self):
        self.running_flag = False
        self.fc.stop_realtime_control()

    def run(self):
        fc = self.fc
        cam = self.cam
        ########## init_value ###########
        self.running_flag = True
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task,
                             daemon=True)  # 在添加到列表的时候就已经完成定义了
        )

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
        self.navigation_to_waypoint(point)  # 初始化路径点
        self.switch_pid("default")
        sleep(0.1)
        self.navigation_flag = True
        self.set_navigation_speed(self.navigation_speed)

    def navigation_to_waypoint(self, waypoint):
        self.pos_x_pid.SetPoint = waypoint[0]
        self.pos_y_pid.SetPoint = waypoint[1]

    def navigation_task(self):
        ######## 解算参数 ########
        SIZE = 1000
        SCALE_RATIO = 0.5
        LOW_PASS_RATIO = 0.6
        ########################
        paused = False
        while self.running:
            sleep(0.01)
            if (
                    self.navigation_flag
                    and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
                if paused:
                    paused = False
                    # 初始化pid
                    self.pos_x_pid.auto_mode = True
                    self.pos_y_pid.auto_mode = True
                    self.yaw_pid.auto_mode = True
                    # 以当前位置为稳定点
                    self.pos_x_pid.setpoint = self.current_pos[1]
                    self.pos_y_pid.setpoint = self.current_pos[2]
                    self.yaw_pid.SetPoint = self.current_pos[3]
                    logger.info("[MISSION] Navigation resumed")
                # 这里是要判断是否开始用点云图判断位置,没有就开启
                if not self.radar._rtpose_flag:
                    self.radar.start_resolve_pose(
                        size=SIZE,
                        scale_ratio=SCALE_RATIO,
                        low_pass_ratio=LOW_PASS_RATIO,
                    )
                    logger.info("[MISSION] Resolve pose started")
                    sleep(0.01)
                # 等待地图更新,等待其被set,将线程阻塞一秒
                if self.radar.rt_pose_update_event.wait(1):
                    self.radar.rt_pose_update_event.clear()
                    current_x = self.radar.rt_pose[0]
                    current_y = self.radar.rt_pose[1]
                    # 由于我没有搞定飞控的坐标系问题，所以这里还是注释掉先
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
                    # if False:  # debug logger.debug( f"[MISSION] Current pose: {current_x}, {current_y},
                    # {current_yaw}; Output: {out_x}, {out_y}, {out_yaw}" )
            else:
                self.radar.stop_resolve_pose()
                if not paused:
                    paused = True
                    self.pos_x_pid.set_auto_mode(False)
                    self.pos_y_pid.set_auto_mode(False)
                    self.yaw_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.info("[MISSION] Navigation paused")
