import cv2 as cv
import time
import numpy as np
from math import sin, cos
import threading
from simple_pid import PID
from time import sleep
from ProtocolMCU.Application import class_application
# from RadarDrivers.RadarDriver import LD_Radar
from others.Logger import logger


class Mission:
    def __init__(self, _fc: class_application, camera: cv.VideoCapture):
        self.fc = _fc
        self.cam = camera
        # self.radar = radar
        self.DEBUG = True
        self.inital_yaw = self.fc.state.yaw.value
        self.cruise_height = 120

        # PID
        self.pid_tunings = {
            "default": (0.35, 0, 0.08),  # 导航
            "delivery": (0.4, 0.05, 0.16),  # 配送
            "landing": (0.4, 0.05, 0.16),  # 降落
        }  # PID参数 (仅导航XY使用) default settings
        self.height_pid = PID(
            0.4,
            0.0,
            0.1,
            setpoint=0,
            output_limits=(-20, 20),
            auto_mode=False,
        )
        self.height_pid.setpoint = 140  # 定高120cm
        self.pos_x_pid = PID(
            0.35,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.pos_y_pid = PID(
            0.35,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.pos_mid_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-15, 15),
            auto_mode=False,
        )
        self.pos_line_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-15, 15),
            auto_mode=False
        )
        self.yaw_pid = PID(
            0.2,
            0.0,
            0.0,
            setpoint=0,
            output_limits=(-45, 45),
            auto_mode=False,
        )

        self.pos_data = [0, 0]
        self.pos_data_init = [False, False]
        self.pos_low_pass_ratio = 0.7
        self.pos_scale_ratio = 1

        self.camera_down_pwm = 32.5
        self.camera_up_pwm = 72

        # FLAGS
        self.running_flag = False  # 运行flag
        self.keep_height_flag = False  # 定高flag
        self.navigation_flag = False
        self.navigation_circle_flag = False
        self.navigation_radar_flag = False
        self.thread_list = []  # 线程列表
        self.navigation_speed = 35  # 导航速度
        self.precision_speed = 25  # 精确速度

    def run_height_task(self):
        _fc = self.fc
        _fc.start_realtime_control(20)
        self.navigation_speed = 35  # 导航速度
        self.precision_speed = 25  # 精确速度

        # 创建线程
        self.running_flag = True
        _fc.start_realtime_control(20)  # 别忘了启动实时控制的线程
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task, daemon=True)
        )
        self.thread_list[-1].start()
        self.thread_list.append(
            threading.Thread(target=self.navigation_task_gps, daemon=True)
        )
        self.thread_list[-1].start()
        self.set_navigation_speed(self.navigation_speed)

        _fc.unlock()
        logger.info("takeoff")
        sleep(2)
        _fc.set_flight_mode(_fc.PROGRAM_MODE)
        sleep(0.5)
        if not _fc.check_mode(_fc.PROGRAM_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.lock()
            sleep(0.5)
            exit()

        _fc.takeoff(100)
        _fc.wait_for_hovering()
        # fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        # sleep(0.5)
        # if not fc.check_mode(fc.HOLD_POS_MODE):
        #     logger.warning("[MISSION] MISTAKE MODE")
        #     fc.land()
        #     sleep(2)
        #     fc.lock()
        #     sleep(0.5)
        #     exit()
        # sleep(0.5)

        # 启动定高，启动位置环
        self.height_pid.setpoint = 100
        self.keep_height_flag = True
        self.pos_x_pid.setpoint = _fc.state.pos_x.value
        self.pos_y_pid.setpoint = _fc.state.pos_y.value
        self.navigation_flag = True
        sleep(13)

        self.height_pid.setpoint = 120
        sleep(10)

        _fc.set_flight_mode(_fc.PROGRAM_MODE)
        sleep(0.5)
        if not _fc.check_mode(_fc.PROGRAM_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.lock()
            sleep(0.5)
            exit()

        _fc.horizontal_move(distance=200, direction=0, speed=25)
        _fc.wait_for_hovering()

        # 降落
        self.keep_height_flag = False
        self.navigation_flag = False
        _fc.set_flight_mode(_fc.PROGRAM_MODE)
        sleep(0.1)
        if not _fc.check_mode(_fc.PROGRAM_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.land()
            sleep(2)
            _fc.lock()
            sleep(0.5)
            exit()
        _fc.land()
        sleep(3.5)
        _fc.lock()

    def run_task_one(self):
        """
        任务1
        完成所有基础的线程的建立
        :return:
        """
        _fc = self.fc
        # 参数
        self.navigation_speed = 35  # 导航速度
        self.precision_speed = 25  # 精确速度

        # 创建线程
        self.running_flag = True
        _fc.start_realtime_control(20)  # 别忘了启动实时控制的线程
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task, daemon=True)
        )
        self.thread_list[-1].start()
        self.thread_list.append(
            threading.Thread(target=self.navigation_task_gps, daemon=True)
        )
        self.thread_list[-1].start()
        self.set_navigation_speed(self.navigation_speed)

        # 起飞
        _fc.unlock()
        logger.info("takeoff")
        sleep(2)
        _fc.set_flight_mode(_fc.PROGRAM_MODE)
        sleep(0.5)
        if not _fc.check_mode(_fc.PROGRAM_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.lock()
            sleep(0.5)
            exit()

        _fc.takeoff(120)
        _fc.wait_for_hovering()
        # fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        # sleep(0.5)
        # if not fc.check_mode(fc.HOLD_POS_MODE):
        #     logger.warning("[MISSION] MISTAKE MODE")
        #     fc.land()
        #     sleep(2)
        #     fc.lock()
        #     sleep(0.5)
        #     exit()
        # sleep(0.5)

        self.height_pid.setpoint = 120
        # 启动定高，启动位置环
        # self.keep_height_flag = True
        # self.pos_x_pid.setpoint = fc.state.pos_x.value
        # self.pos_y_pid.setpoint = fc.state.pos_y.value
        # self.navigation_flag = True
        # sleep(3)

        # 走个正方形
        # self.move_in_rec()
        sleep(5)
        _fc.horizontal_move(distance=90, speed=25, direction=45)
        _fc.wait_for_hovering()
        self.run_task_one_program()

        # 降落
        self.keep_height_flag = False
        self.navigation_flag = False
        _fc.set_flight_mode(_fc.PROGRAM_MODE)
        sleep(0.1)
        if not _fc.check_mode(_fc.PROGRAM_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.land()
            sleep(2)
            _fc.lock()
            sleep(0.5)
            exit()
        _fc.land()
        sleep(3.5)
        _fc.lock()

    def run_task_one_program(self):
        _fc = self.fc
        _fc.horizontal_move(distance=200, direction=0, speed=25)
        _fc.wait_for_hovering()
        _fc.horizontal_move(distance=180, direction=270, speed=25)
        _fc.wait_for_hovering()
        _fc.horizontal_move(distance=200, direction=180, speed=25)
        _fc.wait_for_hovering()
        _fc.horizontal_move(distance=190, direction=90, speed=25)
        _fc.wait_for_hovering()

    def run_task_two(self):
        _fc = self.fc

        # 参数设定
        self.navigation_speed = 25  # 导航速度
        _fc.start_realtime_control(20)

        # 起飞
        _fc.unlock()
        logger.info("takeoff")
        sleep(2)
        _fc.set_flight_mode(_fc.PROGRAM_MODE)
        sleep(0.5)
        if not _fc.check_mode(_fc.PROGRAM_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.lock()
            sleep(0.5)
            exit()

        _fc.takeoff(120)
        _fc.wait_for_hovering()
        _fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        sleep(0.5)
        if not _fc.check_mode(_fc.HOLD_POS_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.land()
            sleep(2)
            _fc.lock()
            sleep(0.5)
            exit()
        sleep(0.5)

        # 启动定高，启动位置环
        self.height_pid.setpoint = 120
        self.keep_height_flag = True
        self.navigation_flag = False
        # 等待稳定
        sleep(2)
        # self.pos_x_pid.setpoint = fc.state.pos_x.value
        # self.pos_y_pid.setpoint = fc.state.pos_y.value
        # self.navigation_flag = True
        # 前往圆心正下方位置

        # 设定当前位置为定点位置
        # self.navigation_flag = True
        # sleep(3)
        # self.move_to_circle_GPS()
        # sleep(3)
        # self.navigation_flag = False
        # sleep(1)
        _fc.set_flight_mode(_fc.PROGRAM_MODE)
        sleep(0.5)
        if not _fc.check_mode(_fc.PROGRAM_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.lock()
            sleep(0.5)
            exit()
        sleep(0.1)

        _fc.horizontal_move(direction=0, distance=150, speed=25)
        sleep(3)
        _fc.wait_for_hovering()

        _fc.set_flight_mode(_fc.HOLD_POS_MODE)
        sleep(0.5)
        if not _fc.check_mode(_fc.HOLD_POS_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.lock()
            sleep(0.5)
            exit()
        sleep(2)
        self.move_in_circle_openloop()

        # 降落
        self.keep_height_flag = True
        # self.navigation_flag = False
        sleep(0.1)

        if not _fc.check_mode(_fc.PROGRAM_MODE):
            logger.warning("[MISSION] MISTAKE MODE")
            _fc.land()
            sleep(2)
            _fc.lock()
            sleep(0.5)
            exit()
        _fc.land()
        sleep(4.5)
        _fc.lock()

    def set_navigation_speed(self, speed):
        speed = abs(speed)
        self.pos_x_pid.output_limits = (-speed, speed)
        self.pos_y_pid.output_limits = (-speed, speed)

    def stop(self):
        self.running_flag = False
        self.fc.stop_realtime_control()

    def keep_height_task(self):
        # 注意，这些线程的初始化必须加上保护，paused就是保护措施，让线程经历初始化
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
                # if self.DEBUG:  # debug
                #     logger.info(
                #         f"[MISSION] Current height: {self.fc.state.alt_add.value}; Output: {out_height}"
                #     )
            else:
                if not paused:
                    paused = True
                    self.height_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_z=0)
                    logger.info("[MISSION] Keep height paused")

    def navigation_task_gps(self):
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
                    # 记录当前位置
                    self.pos_data[0] = self.fc.state.pos_x.value
                    self.pos_data[1] = self.fc.state.pos_y.value
                    logger.info("[MISSION] Navigation resumed")
                    sleep(0.01)

                current_x = self.fc.state.pos_x.value
                current_y = self.fc.state.pos_y.value
                out_x = None
                out_y = None
                if current_x is not None:
                    if self.pos_data_init[0]:  # 判断是否被初始化过
                        self.pos_data[0] += (
                                                    current_x / self.pos_scale_ratio -
                                                    self.pos_data[0]) * self.pos_low_pass_ratio
                    else:
                        self.pos_data[0] = current_x / self.pos_scale_ratio
                        self.pos_data_init[0] = True
                if current_y is not None:
                    if self.pos_data_init[1]:
                        self.pos_data[1] += (
                                                    current_y / self.pos_scale_ratio -
                                                    self.pos_data[1]) * self.pos_low_pass_ratio
                    else:
                        self.pos_data[1] = current_y / self.pos_scale_ratio
                        self.pos_data_init[1] = True

                # 更新实时控制
                pos_x_now = self.pos_data[0]
                pos_y_now = self.pos_data[1]
                if pos_x_now is not None:
                    out_x = int(self.pos_x_pid(pos_x_now))
                    if out_x is not None:
                        self.fc.update_realtime_control(vel_x=out_x)
                if pos_y_now is not None:
                    out_y = int(self.pos_y_pid(pos_y_now))
                    if out_y is not None:
                        self.fc.update_realtime_control(vel_y=out_y)
                if self.DEBUG:  # debug
                    logger.info(
                        f"[MISSION] Current pose: {current_x}, {current_y}; "
                        f"Output: {out_x}, {out_y}; "
                        f"Setpoint: {self.pos_x_pid.setpoint}, {self.pos_y_pid.setpoint}"
                    )
            else:
                if not paused:
                    paused = True
                    self.pos_x_pid.set_auto_mode(False)
                    self.pos_y_pid.set_auto_mode(False)
                    self.pos_data_init = [False, False]
                    self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.info("[MISSION] Navigation paused")

    # def navigation_task_radar_find_nearest(self):
    #     radar = self.radar
    #     paused = False
    #     while self.running_flag:
    #         if (self.fc.state.mode.value == fc.HOLD_POS_MODE,
    #             self.navigation_radar_flag):
    #             if paused:
    #             point_array = radar.map.find_two_point_with_given_distance()
    #             point_one = point_array[0]
    #             point_two = point_array[1]
    #             p_one_x, p_one_y = point_one.to_xy()
    #             p_two_x, p_two_y = point_two.to_xy()
    #             average_y = int((p_one_y + p_two_y) / 2)

    def program_move_with_pos(self, dis_x=0, dis_y=0):
        if self.navigation_flag:
            current_pos_x = self.pos_data[0]
            current_pos_y = self.pos_data[1]
            self.pos_x_pid.setpoint = current_pos_x + int(dis_x)
            self.pos_y_pid.setpoint = current_pos_y + int(dis_y)
        else:
            logger.warning("[MISSION] Navigation task not started")
            return False
        result = self.wait_for_waypoint()
        return result

    def wait_for_waypoint(self, time_thres=1.5, pos_thres=15, timeout=20):
        time_count = 0
        time_start = time.time()
        while True:
            sleep(0.1)
            if self.reached_waypoint(pos_thres):
                time_count += 0.1
            if time_count >= time_thres:
                logger.info("[MISSION] Reached waypoint")
                return True
            if time.time() - time_start > timeout:
                logger.warning("[MISSION] Waypoint overtime")
                return False

    def reached_waypoint(self, pos_thres=10):
        return (
                abs(self.pos_data[0] - self.pos_x_pid.setpoint) < pos_thres
                and abs(self.pos_data[1] - self.pos_y_pid.setpoint) < pos_thres
        )

    def move_in_rec(self):
        result = self.program_move_with_pos(dis_x=200)
        if not result:
            logger.warning("[MISSION] Mission fail, program move error")
        sleep(2)
        result = self.program_move_with_pos(dis_y=200)
        if not result:
            logger.warning("[MISSION] Mission fail, program move error")
        sleep(2)
        result = self.program_move_with_pos(dis_x=-200)
        if not result:
            logger.warning("[MISSION] Mission fail, program move error")
        sleep(2)
        result = self.program_move_with_pos(dis_y=-200)
        if not result:
            logger.warning("[MISSION] Mission fail, program move error")
        sleep(2)

    def point_landing(self):
        pass

    def move_forward_to_find_circle(self):
        _cam = self.cam
        start_time = time.time()
        find_circle, circle = self.find_circle()
        if not find_circle:
            while not find_circle:
                sleep(0.01)
                current_time = time.time()
                if current_time - start_time > 10:
                    logger.warning("[MISSION] Find Circle Failed, Time Out")
                    self.fc.update_realtime_control(vel_x=0, vel_y=0)
                    return False
                self.fc.update_realtime_control(vel_y=20)
                find_circle, circle = self.find_circle()
        self.fc.update_realtime_control(vel_x=0, vel_y=0)
        return True

    def move_to_bottom_of_circle(self):
        _cam = self.cam
        find_circle, circle = self.find_circle()
        if not find_circle:
            logger.warning("[MISSION] No Circle Found In Move To Circle Bottom")
            return False

        pos_thres = 10

        start_time = time.time()

        while self.running_flag:
            sleep(0.01)
            out_x = None
            out_y = None
            find_circle, circle = self.find_circle()
            current_x = circle[1]
            current_y = circle[0]
            if current_x is not None:
                out_x = -int(self.pos_x_pid(current_x))
                if out_x is not None:
                    self.fc.update_realtime_control(vel_x=out_x)
            if current_y is not None:
                out_y = -int(self.pos_y_pid(current_y))
                if out_y is not None:
                    self.fc.update_realtime_control(vel_y=out_y)
            if self.DEBUG:  # debug
                logger.info(
                    f"[MISSION] Current pose: {current_x}, {current_y};"
                    f"Output: {out_x}, {out_y};"
                    f"Setpoint: {self.pos_x_pid.setpoint}, {self.pos_y_pid.setpoint}"
                )

            if (
                    abs(current_x - self.pos_x_pid.setpoint) < pos_thres
                    and abs(current_y - self.pos_y_pid.setpoint) < pos_thres
            ):
                self.fc.update_realtime_control(vel_x=0, vel_y=0)
                return True

            current_time = time.time()
            if current_time - start_time > 10:
                logger.warning("[MISSION] Move To Circle Bottom Failed, Time Out")
                self.fc.update_realtime_control(vel_x=0, vel_y=0)
                return False

    def move_to_circle_gps(self):
        result = self.program_move_with_pos(dis_x=100, dis_y=100)
        if not result:
            logger.warning("[MISSION] Mission fail, program move error")
        sleep(2)

    def move_in_circle_openloop(self):
        start_time = time.time()
        current_time = start_time
        while current_time - start_time < 24:
            current_time = time.time()
            time_now = current_time - start_time
            vel_x = int(sin(time_now / 4) * 20)
            vel_y = int(cos(time_now / 4) * 20)
            self.fc.update_realtime_control(vel_x=vel_x, vel_y=vel_y)
        self.fc.update_realtime_control(vel_x=0, vel_y=0)

    # def move_in_circle_GPS(self):
    #     cam = self.cam
    #     fc = self.fc
    #     circle_point_x = self.pos_data[0] + 100
    #     circle_point_y = self.pos_data[1]
    #     paused = True
    #
    #     while self.running_flag:
    #         sleep(0.01)
    #         if (
    #                 self.navigation_circle_flag
    #                 and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
    #         ):
    #
    #             if paused:
    #                 paused = False
    #                 self.pos_x_pid.set_auto_mode(True, last_output=0)
    #                 self.pos_y_pid.set_auto_mode(True, last_output=0)
    #                 # 记录当前位置
    #                 self.pos_data[0] = self.fc.state.pos_x.value
    #                 self.pos_data[1] = self.fc.state.pos_y.value
    #                 logger.info("[MISSION] Navigation resumed")
    #                 sleep(0.01)
    #
    #             current_dis_x = self.fc.state.pos_x.value - circle_point_x
    #             current_mid = self.fc.state.pos_x.value
    #             current_y = self.fc.state.pos_y.value
    #             out_x = None
    #             out_y = None
    #             if current_mid is not None:
    #                 if self.pos_data_init[0]:  # 判断是否被初始化过
    #                     self.pos_data[0] += (
    #                                                 current_mid / self.pos_scale_ratio -
    #                                                 self.pos_data[0]) * self.pos_low_pass_ratio
    #                 else:
    #                     self.pos_data[0] = current_mid / self.pos_scale_ratio
    #                     self.pos_data_init[0] = True
    #             if current_sin_theta is not None:
    #                 if self.pos_data_init[1]:
    #                     self.pos_data[1] += (
    #                                                 current_sin_theta / self.pos_scale_ratio -
    #                                                 self.pos_data[1]) * self.pos_low_pass_ratio
    #                 else:
    #                     self.pos_data[1] = current_y / self.pos_scale_ratio
    #                     self.pos_data_init[1] = True
    #
    #             # 更新实时控制
    #             pos_x_now = self.pos_data[0]
    #             pos_y_now = self.pos_data[1]
    #             if pos_x_now is not None:
    #                 out_x = int(self.pos_x_pid(pos_x_now))
    #                 if out_x is not None:
    #                     self.fc.update_realtime_control(vel_x=out_x)
    #             if pos_y_now is not None:
    #                 out_y = int(self.pos_y_pid(pos_y_now))
    #                 if out_y is not None:
    #                     self.fc.update_realtime_control(vel_y=out_y)
    #             # if self.DEBUG:  # debug
    #             #     logger.info(
    #                     f"[MISSION] Current pose: {current_x}, {current_y};"
    #                     f"Output: {out_x}, {out_y};"
    #                     f"Setpoint: {self.pos_x_pid.setpoint}, {self.pos_y_pid.setpoint}"
    #             #     )
    #         else:
    #             if not paused:
    #                 paused = True
    #                 self.pos_x_pid.set_auto_mode(False)
    #                 self.pos_y_pid.set_auto_mode(False)
    #                 self.pos_data_init = [False, False]
    #                 self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
    #                 logger.info("[MISSION] Navigation paused")

    def find_circle(self):
        _cam = self.cam
        ret, frame = _cam.read()
        # cv.imshow('frame', frame)
        img = frame

        result = [0, 0]
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        median = cv.medianBlur(gray, 7)
        # 霍夫圆检测
        circle = cv.HoughCircles(median, cv.HOUGH_GRADIENT, 1, 500, param1=100, param2=100, minRadius=50,
                                 maxRadius=1000)
        if circle is not None:
            circle = np.uint16(np.around(circle))
            # print(circle)
            # 将检测结果绘制在图像上
            for i in circle[0, :]:  # 遍历矩阵的每一行的数据
                cv.circle(img, (i[0], i[1]), i[2], (255, 0, 0), 10)
                # 绘制圆心
                cv.circle(img, (i[0], i[1]), 10, (255, 0, 0), -1)
                print(i[0], i[1])
                result = i
        else:
            return False, None
        cv.imshow("final", img)
        return True, result


if __name__ == "__main__":
    fc = class_application()
    cam = cv.VideoCapture(0)
    cam.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cam.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    fourcc = cv.VideoWriter_fourcc('M', 'J', 'P', 'G')
    cam.set(cv.CAP_PROP_FOURCC, fourcc)
    cam.set(cv.CAP_PROP_FPS, 30)
    if not cam.isOpened():
        cam.open(0)
    assert cam.isOpened()
    mission = Mission(fc, cam)
    # logger.warning("[MANAGER] Camera Opening Failed")
    while True:
        sleep(0.001)
        mission.find_circle()
        cv.waitKey(100)
