import cv2 as cv
import threading
from simple_pid import PID
from RadarDrivers_reconstruct.Radar import Radar
from ProtocolMCU.Application import class_application
from others.Logger import logger
from time import sleep, time

point_a = [275, 380]

point_array_test = [[145, 165],
                    [295, 165],
                    [300, 315],
                    [145, 315]]

point_array = [[97, 65], [97, 116], [97, 164], [97, 210],
               [147, 65], [147, 116], [147, 164], [147, 210],
               [197, 164], [197, 210], [247, 65], [247, 116],
               [247, 164], [247, 210], [295, 65], [295, 116],
               [295, 164], [295, 210], [295, 262], [297, 315],
               [295, 360], [344, 65], [344, 116], [344, 164],
               [344, 210], [344, 262], [344, 315], [344, 360]]


class Mission:
    def __init__(self, fc: class_application, camera: cv.VideoCapture, radar: Radar):
        '''
        完成基本的实例的定义
        完成底层初始化
        完成PID的初始化(实时控制的四个维度控制PID)
        '''
        self.fc = fc
        self.cam = camera
        self.radar = radar
        self.DEBUG = True

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
        self.height_pid.setpoint = 140  # 定高140cm
        self.pos_x_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=True,
        )
        self.pos_y_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=True,
        )
        self.yaw_pid = PID(
            0.2,
            0.0,
            0.0,
            setpoint=0,
            output_limits=(-45, 45),
            auto_mode=True,
        )
        self.yaw_pid.setpoint = 0

        ######### FLAGS #########
        self.keep_height_flag = False   # 定高flag
        self.running_flag = False   # 运行flag
        self.thread_list = []    # 线程列表
        self.navigation_flag = False
        self.navigation_speed = 27  # 导航速度
        self.precision_speed = 20  # 精确速度
        # self.camera_down_pwm = 32.5
        # self.camera_up_pwm = 72
        self.cruise_height = 125  # 巡航高度

    # 建立run函数
    def run(self):
        fc = self.fc
        # cam = self.cam
        radar = self.radar
        ########### 参数#########
        self.running_flag = True
        ################ 启动线程 ################
        self.running = True
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task, daemon=True)
        )
        self.thread_list[-1].start()
        self.thread_list.append(
            threading.Thread(target=self.navigation_task_radar, daemon=True)
        )
        self.thread_list[-1].start()
        sleep(4)
        logger.info("[MISSION] Threads started")

        ################ 初始化 ################
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.set_navigation_speed(self.navigation_speed)
        self.fc.start_realtime_control(20)  # 频率20Hz
        self.switch_pid("default")
        BASE_POINT = [self.radar.rt_pose[0], self.radar.rt_pose[1]]
        LANDING_POINT = BASE_POINT
        logger.info(f"[LADAR] BASE_POINT Set {BASE_POINT}")

        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
        sleep(1)
        self.point_takeoff(BASE_POINT)
        logger.info("[MISSION] HOLDING POSE")
        sleep(2)    # 定点自稳测试

        ################ 开始任务 ################
        self.set_navigation_speed(self.navigation_speed)
        # 前往A点
        logger.info("[MISSION] Navigate to Point_a")
        self.navigation_to_waypoint(point_a)
        self.wait_for_waypoint()
        sleep(2)

        # 开始按照点一个一个前往
        # for target_point in point_array_test:
        #     logger.info(f"[MISSION] Navigate to {target_point}")
        #     self.navigation_to_waypoint(target_point)
        #     self.wait_for_waypoint()
        #     sleep(5)    # 定点自稳测试

        # # 植保任务
        for target_point in point_array:
            logger.info(f"[MISSION] Navigate to {target_point}")
            self.navigation_to_waypoint(target_point)
            self.wait_for_waypoint()
            sleep(0.5)

        # 回到A点
        logger.info("[MISSION] Go Back to Point_a")
        self.navigation_to_waypoint(point_a)
        self.wait_for_waypoint()
        sleep(2)

        # 回到基地点
        logger.info("[MISSION] Go to Base")
        self.navigation_to_waypoint(BASE_POINT)
        self.wait_for_waypoint()

        # 降落
        self.pointing_landing(LANDING_POINT)
        logger.info("[MISSION] Misson-1 Finished")

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
        # inital_yaw = self.fc.state.yaw.value
        sleep(2)  # 等待电机启动
        self.fc.takeoff(140)
        self.fc.wait_for_takeoff_done()
        # self.fc.set_yaw(inital_yaw, 15)
        self.fc.wait_for_hovering(2)
        # 闭环定高
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        self.height_pid.setpoint = self.cruise_height
        self.keep_height_flag = True
        sleep(2)
        self.navigation_to_waypoint(point)  # 设置路径点(认为是当前位置)
        self.switch_pid("default")
        sleep(0.1)
        self.navigation_flag = True
        self.set_navigation_speed(self.precision_speed)

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
        self.height_pid.setpoint = 80
        sleep(1.5)
        self.height_pid.setpoint = 40
        sleep(1.5)
        self.height_pid.setpoint = 20
        sleep(3)
        self.wait_for_waypoint()
        # self.height_pid.setpoint = 0
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        sleep(0.1)
        self.fc.land()
        self.fc.wait_for_lock(5)
        self.fc.lock()

    def navigation_to_waypoint(self, waypoint):
        self.pos_x_pid.setpoint = waypoint[0]
        self.pos_y_pid.setpoint = waypoint[1]

    def set_navigation_speed(self, speed):
        speed = abs(speed)
        self.pos_x_pid.output_limits = (-speed, speed)
        self.pos_y_pid.output_limits = (-speed, speed)

    def wait_for_waypoint(self, time_thres=1, pos_thres=10, timeout=30):
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
        SCALE_RATIO = 1
        LOW_PASS_RATIO = 0.6
        ########################
        paused = True
        # 建议保持位置解算持续运行，直到视觉介入再pause(需要单独写一条)
        if not self.radar._rtpose_flag:
            self.radar.start_resolve_pose(
                size=SIZE,
                scale_ratio=SCALE_RATIO,
                low_pass_ratio=LOW_PASS_RATIO,
            )
            logger.info("[MISSION] Resolve pose started")
            sleep(1)
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
                    sleep(1)

                # 等待地图更新,这个event是给这个线程用的
                if self.radar.rt_pose_update_event.wait(1):

                    self.radar.rt_pose_update_event.clear()   # 地图更新，重新上锁，等待再次set

                    current_x = self.radar.rt_pose[0]
                    current_y = self.radar.rt_pose[1]
                    current_yaw = self.radar.rt_pose[2]
                    out_x = 0
                    out_y = 0
                    out_yaw = 0

                    if current_x > 0:  # 0 为无效值
                        out_x = int(self.pos_x_pid(current_x))
                        if out_x is not None:
                            pass
                            self.fc.update_realtime_control(vel_x=out_x)
                    if current_y > 0:
                        out_y = int(self.pos_y_pid(current_y))
                        if out_y is not None:
                            pass
                            self.fc.update_realtime_control(vel_y=out_y)
                    if current_yaw is not None:
                        out_yaw = int(self.yaw_pid(current_yaw))
                        if out_yaw is not None:
                            pass
                            self.fc.update_realtime_control(yaw=out_yaw)
                    if self.DEBUG:  # debug
                        logger.info(
                            f"[MISSION] Current pose: {current_x}, {current_y}, {current_yaw}; Output: {out_x}, {out_y}, {out_yaw}"
                        )
            else:
                if not paused:
                    paused = True
                    self.pos_x_pid.set_auto_mode(False)
                    self.pos_y_pid.set_auto_mode(False)
                    self.yaw_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.info("[MISSION] Navigation paused")
