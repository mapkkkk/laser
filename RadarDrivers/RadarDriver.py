import threading
import time
import struct

import cv2
import numpy as np
import serial

from others.Logger import logger
from RadarDrivers.RadarVisualResolver import radar_resolve_rt_pose
from RadarDrivers.RadarDataStruct import Map_360, Point_2D, Radar_Package
from RadarDrivers.DriverComponents import calculate_crc8
"""
为了搞懂这部分代码，有几个知识点必须知道
线程同步的Event还有基本串口通讯的理论
HZW 正在一点点补足注释内容
23.4.3 注释基本补足
"""

# > This class is used to create a radar object that can be used to plot radar data


class LD_Radar(object):
    """
    雷达是双线程的模块，包括串口接收线程和地图解析线程，两个线程间的同步靠的是线程锁event
    """
    def __init__(self):
        self.running = False
        self.thread_list = []
        self._package = Radar_Package()
        self._serial = None
        self._update_callback = None
        self._map_updated_event = threading.Event()
        self.rt_pose_update_event = threading.Event()
        self._fp_flag = False
        self._rtpose_flag = False
        self.fp_points = []
        self.rt_pose = [0, 0, 0]
        self._rt_pose_inited = [False, False, False]
        self.map = Map_360()
        self._radar_unpack_fmt = "<HH" + "HB" * 12 + "HH"  # 雷达数据解析格式

    def start(self, com_port, radar_type: str = "LD06", update_callback=None):
        """
        开始监听雷达数据
        radar_type: LD08 or LD06
        update_callback: 回调函数，每次更新雷达数据时调用
        """
        if self.running:
            self.stop()
        if radar_type == "LD08":
            baudrate = 115200
        elif radar_type == "LD06":
            baudrate = 230400
        else:
            raise ValueError("Unknown radar type")
        self._serial = serial.Serial(com_port, baudrate=baudrate)
        self._update_callback = update_callback
        self.running = True
        thread = threading.Thread(target=self._read_serial_task)
        thread.daemon = True
        thread.start()
        self.thread_list.append(thread)
        logger.info("[RADAR] Listenning thread started")
        thread = threading.Thread(target=self._map_resolve_task)
        thread.daemon = True    # 守护线程的意思就是在这个线程运行时，主线程的退出不会导致整个程序的退出，会等到所有守护线程结束
        thread.start()
        self.thread_list.append(thread)
        logger.info("[RADAR] Map resolve thread started")

    def stop(self, joined=False):
        """
        停止监听雷达数据
        """
        self.running = False
        if joined:
            for thread in self.thread_list:
                thread.join()   # 日常解释join():认为是让主线程等待其执行完，其实就是在没执行完其中一个线程时，这句话就不会继续往下走
        if self._serial != None:
            self._serial.close()
        logger.info("[RADAR] Stopped all threads")

    def _read_serial_task(self):
        """
        基本串口操作，大部分备注留在Serial文件里了
        """
        reading_flag = False
        start_bit = b"\x54\x2C"
        package_length = 45
        read_buffer = bytes()
        wait_buffer = bytes()
        while self.running:
            try:
                if self._serial.in_waiting > 0:
                    if not reading_flag:  # 等待包头
                        wait_buffer += self._serial.read(1)
                        if len(wait_buffer) >= 2:
                            if wait_buffer[-2:] == start_bit:
                                reading_flag = True
                                read_count = 0
                                wait_buffer = bytes()
                                read_buffer = start_bit
                    else:  # 读取数据
                        read_buffer += self._serial.read(package_length)
                        reading_flag = False
                        # 直接传入写在RadarResolver中的解析函数解析(这里之前写的有问题（但可能是我有问题））
                        self._package = self.resolve_radar_data(read_buffer, self._package)
                        self.map.update(self._package)
                        self._map_updated_event.set()   # 保证有输入之后才更新地图，减少占用，其实可以理解为两个线程之间的同步过程
                        if self._update_callback != None:
                            self._update_callback()
                else:
                    time.sleep(0.001)
                if self._fp_flag:
                    self.check_target_point()
            except Exception as e:
                logger.error(f"[RADAR] Listenning thread error: {e}")
                time.sleep(0.5)

    def resolve_radar_data(self, data: bytes, to_package: Radar_Package = None) -> Radar_Package:
        """
        解析雷达原始数据
        data: bytes 原始数据
        to_package: 传入一个RadarPackage对象, 如果不传入, 则会新建一个
        return: 解析后的RadarPackage对象
        """
        if len(data) != 47:  # fixed length of radar data
            logger.warning(f"[RADAR] Invalid data length: {len(data)}")
            return None
        if calculate_crc8(data[:-1]) != data[-1]:
            logger.warning("[RADAR] Invalid CRC8")
            return None
        if data[:2] != b"\x54\x2C":
            logger.warning(f"[RADAR] Invalid header: {data[:2]:X}")
            return None
        datas = struct.unpack(self._radar_unpack_fmt, data[2:-1])
        if to_package is None:
            return Radar_Package(datas)
        else:
            to_package.fill_data(datas)
            return to_package
    def _map_resolve_task(self):
        while self.running:
            try:
                # 这里是这个意思：若event是False，就停，等待True，但是是True就直接过，返回True，timeout之后将无视False直接返回
                if self._map_updated_event.wait(1):
                    self._map_updated_event.clear()     # 这里则是将event设置为False，下一次循环又会再次等待
                    # 是否查找杆的位置，有两种类型的找杆
                    if self._fp_flag:
                        if self._fp_type == 0:
                            self.update_target_point(
                                self.map.find_nearest(*self._fp_arg)
                            )
                        elif self._fp_type == 1:
                            self.update_target_point(
                                self.map.find_nearest_with_ext_point_opt(
                                    *self._fp_arg)
                            )
                    # 位置解析，注意函数的对应内容
                    # 注意这里的位置信息更新处理，采用了低通滤波的形式（学长语），通俗解释是进行了一定程度上的削峰处理
                    # 三轴解析（x, y, yaw)，z轴有激光
                    if self._rtpose_flag:
                        img = self.map.output_cloud(
                            size=int(self._rtpose_size),
                            scale=0.1 * self._rtpose_scale_ratio,
                        )
                        x, y, yaw = radar_resolve_rt_pose(img)  # 解析点云图，用的还是cv其实
                        if x is not None:
                            if self._rt_pose_inited[0]:     # 判断是否被初始化过
                                self.rt_pose[0] += (
                                    x / self._rtpose_scale_ratio -
                                    self.rt_pose[0]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[0] = x / self._rtpose_scale_ratio
                                self._rt_pose_inited[0] = True
                        if y is not None:
                            if self._rt_pose_inited[1]:
                                self.rt_pose[1] += (
                                    y / self._rtpose_scale_ratio -
                                    self.rt_pose[1]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[1] = y / self._rtpose_scale_ratio
                                self._rt_pose_inited[1] = True
                        if yaw is not None:
                            if self._rt_pose_inited[2]:
                                self.rt_pose[2] += (
                                    yaw - self.rt_pose[2]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[2] = yaw
                                self._rt_pose_inited[2] = True
                        self.rt_pose_update_event.set()
                else:
                    logger.warning("[RADAR] Map resolve thread wait timeout")
            except Exception as e:
                import traceback
                logger.error(
                    f"[RADAR] Map resolve thread error: {traceback.format_exc()}"
                )
                time.sleep(0.5)

    def init_radar_map(self):
        """
        It creates a radar map image, draws the axes and circles on it, and sets up a mouse callback
        function to display the radar map
        """
        self._radar_map_img = np.zeros((600, 600, 3), dtype=np.uint8)
        a = np.sqrt(2) * 600
        b = (a - 600) / 2
        c = a - b
        b = int(b / np.sqrt(2))
        c = int(c / np.sqrt(2))
        cv2.line(self._radar_map_img, (b, b), (c, c), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (c, b), (b, c), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (300, 0), (300, 600), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (0, 300), (600, 300), (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 100, (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 200, (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 300, (255, 0, 0), 1)
        self.__radar_map_img_scale = 1
        self.__radar_map_info_angle = -1
        cv2.namedWindow("Radar Map", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(
            "Radar Map",
            lambda *args, **kwargs: self.show_radar_map_on_mouse(
                *args, **kwargs),
        )

    def show_radar_map_on_mouse(self, event, x, y, flags):
        """
        :param event: The event that happened
        :param x: x coordinate of the mouse event
        :param y: The y-coordinate of the mouse event
        :param flags:
        """
        if event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:
                self.__radar_map_img_scale *= 1.1
            else:
                self.__radar_map_img_scale *= 0.9
            self.__radar_map_img_scale = min(
                max(0.001, self.__radar_map_img_scale), 2)
        elif event == cv2.EVENT_LBUTTONDOWN or (
            event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON
        ):
            self.__radar_map_info_angle = (
                90 - np.arctan2(300 - y, x - 300) * 180 / np.pi
            ) % 360
            self.__radar_map_info_angle = int(self.__radar_map_info_angle)

    def show_radar_map(self):
        """
        显示雷达地图(调试用, 高占用且阻塞)
        """
        self.init_radar_map()
        while True:
            img_ = self._radar_map_img.copy()
            cv2.putText(
                img_,
                f"{100/self.__radar_map_img_scale:.0f}",
                (300, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            cv2.putText(
                img_,
                f"{200/self.__radar_map_img_scale:.0f}",
                (300, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            cv2.putText(
                img_,
                f"{300/self.__radar_map_img_scale:.0f}",
                (300, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            add_p = self.map.find_two_point_with_given_distance(
                from_=-60,
                to_=60,
                distance=110,
                threshold=15,
                range_limit=3000,
            )
            print(add_p)
            if self.__radar_map_info_angle != -1:
                cv2.putText(
                    img_,
                    f"Angle: {self.__radar_map_info_angle}",
                    (10, 540),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                cv2.putText(
                    img_,
                    f"Distance: {self.map.get_distance(self.__radar_map_info_angle)}",
                    (10, 560),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                point = self.map.get_point(self.__radar_map_info_angle)
                xy = point.to_xy()
                cv2.putText(
                    img_,
                    f"Position: ( {xy[0]:.2f} , {xy[1]:.2f} )",
                    (10, 580),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                add_p = [point] + add_p
                pos = point.to_cv_xy() * self.__radar_map_img_scale + np.array(
                    [300, 300]
                )
                cv2.line(img_, (300, 300),
                         (int(pos[0]), int(pos[1])), (255, 255, 0), 1)
            self.map.draw_on_cv_image(
                img_, scale=self.__radar_map_img_scale, add_points=add_p
            )
            cv2.imshow("Radar Map", img_)
            key = cv2.waitKey(int(1000 / 50))
            if key == ord("q"):
                break
            elif key == ord("w"):
                self.__radar_map_img_scale *= 1.1
            elif key == ord("s"):
                self.__radar_map_img_scale *= 0.9
            elif key == ord("a"):
                out = self.map.output_cloud()
                cv2.imwrite(f"radar_map.png", out)
                cv2.imshow("Cloud", out)

    def start_find_point(
        self,
        timeout: float,
        type: int,
        from_: int,
        to_: int,
        num: int,
        range_limit: int,
    ):
        """
        开始更新目标点
        timeout: 超时时间, 超时后fp_timeout_flag被置位
        type: 0:直接搜索 1:极值搜索
        其余参数与find_nearest一致
        """
        self._fp_update_time = time.time()
        self._fp_timeout = timeout
        self.fp_timeout_flag = False
        self.fp_points = []
        self._fp_flag = True
        self._fp_type = type
        self._fp_arg = (from_, to_, num, range_limit)

    def update_find_point_args(self, from_, to_, num, range_limit):
        """
        更新目标点查找参数
        与find_nearest一致
        """
        self._fp_arg = (from_, to_, num, range_limit)

    def stop_find_point(self):
        """
        停止更新目标点
        """
        self._fp_flag = False

    def update_target_point(self, points: list[Point_2D]):
        """
        更新目标点位置
        """
        if self.fp_timeout_flag and len(points) > 0:
            self.fp_timeout_flag = False
        elif len(points) > 0:
            self.fp_points = points
            self._fp_update_time = time.time()

    def check_target_point(self):
        """
        目标点超时判断
        """
        if (
            not self.fp_timeout_flag
            and time.time() - self._fp_update_time > self._fp_timeout
        ):
            self.fp_timeout_flag = True
            logger.warning("[Radar] lost point!")

    def start_resolve_pose(
        self, size: int = 1000, scale_ratio: float = 1, low_pass_ratio: float = 0.5
    ):
        """
        开始使用点云图解算位姿
        size: 解算范围(长宽为size的正方形)
        scale_ratio: 降采样比例, 降低精度节省计算资源
        low_pass_ratio: 低通滤波比例
        """
        self._rtpose_flag = True
        self._rtpose_size = size
        self._rtpose_scale_ratio = scale_ratio
        self._rtpose_low_pass_ratio = low_pass_ratio
        self.rt_pose = [0, 0, 0]
        self._rt_pose_inited = [False, False, False]

    def stop_resolve_pose(self):
        """
        停止使用点云图解算位姿
        """
        self._rtpose_flag = False
        self.rt_pose = [0, 0, 0]
        self._rt_pose_inited = [False, False, False]
        self.rt_pose_update_event.clear()

    def update_resolve_pose_args(
        self, size: int = 1000, ratio: float = 1, low_pass_ratio: float = 0.5
    ):
        """
        更新参数
        size: 解算范围(长宽为size的正方形)
        scale_ratio: 降采样比例, 降低精度节省计算资源
        low_pass_ratio: 低通滤波比例
        """
        self._rtpose_size = size
        self._rtpose_scale_ratio = ratio
        self._rtpose_low_pass_ratio = low_pass_ratio


if __name__ == "__main__":
    radar = LD_Radar()
    radar.start("/dev/ttyUSB0", "LD06")
    radar.show_radar_map()
    cv2.destroyAllWindows()
