import struct
import time
from typing import List, Optional, Tuple
import cv2
import numpy as np
from scipy.signal import find_peaks
from others.Logger import logger


"""
雷达点云图解决方案，输出所需的图像
觉得量大？那肯定啊，我也觉得——HZW
"""


class Point_2D:
    """
    有大量的object的类函数
    这个类作为点云图里的每个点的存在
    后面的map是这个点的集合
    这个类是作为数据存在的，其中的方法是为了数据服务的
    参考之前的Data里的c_like类型，那个叫Byte啥的东西
    """
    degree = 0.0  # 0.0 ~ 359.9, 0 指向前方, 顺时针
    distance = 0  # 距离 mm
    confidence = 0  # 置信度 典型值=200

    def __init__(self, degree=0, distance=0, confidence=None):
        self.degree = degree
        self.distance = distance
        self.confidence = confidence

    def __str__(self):
        """
        对象描述函数，作用：返回当前对象的字符串类型的信息描述，一般用于对象的直接输出显示
        设置print(obj)打印的信息，默认是对象的内存地址等信息
        :return: s
        """
        s = f"Point: deg = {self.degree:>6.2f}, dist = {self.distance:>4.0f}"
        if self.confidence is not None:
            s += f", conf = {self.confidence:>3.0f}"
        return s

    def __repr__(self):
        """
        类似__str__，只是该函数是面向解释器的。
        """
        return self.__str__()

    def to_xy(self) -> np.ndarray:
        """
        转换到匿名坐标系下的坐标
        """
        return np.array(
            [
                self.distance * np.cos(self.degree * np.pi / 180),
                -self.distance * np.sin(self.degree * np.pi / 180),
            ]
        )

    def to_cv_xy(self) -> np.ndarray:
        """
        转换到OpenCV坐标系下的坐标
        """
        return np.array(
            [
                self.distance * np.sin(self.degree * np.pi / 180),
                -self.distance * np.cos(self.degree * np.pi / 180),
            ]
        )

    def from_xy(self, xy: np.ndarray):
        """
        从匿名坐标系下的坐标转换到点
        """
        self.degree = np.arctan2(-xy[1], xy[0]) * 180 / np.pi % 360
        self.distance = np.sqrt(xy[0] ** 2 + xy[1] ** 2)

    def from_cv_xy(self, xy: np.ndarray):
        """
        从OpenCV坐标系下的坐标转换到点
        """
        self.degree = np.arctan2(xy[0], -xy[1]) * 180 / np.pi % 360
        self.distance = np.sqrt(xy[0] ** 2 + xy[1] ** 2)

    def __eq__(self, __o: object) -> bool:
        """
        判断是否相等 equal ，在obj==other时调用。如果重写了__eq__方法，则会将__hash__方法置为None
        :param __o:
        :return:
        """
        if not isinstance(__o, Point_2D):
            return False
        return self.degree == __o.degree and self.distance == __o.distance

    def to_180_degree(self) -> float:
        """
        转换到-180~180度
        """
        if self.degree > 180:
            return self.degree - 360
        return self.degree

    def __add__(self, other):
        """
        实现了两个对象的加法运算,似乎c++课一定会讲的
        :param other:
        :return: 返回的是一个加好的对象。。。
        """
        if not isinstance(other, Point_2D):
            raise TypeError("Point_2D can only add with Point_2D")
        return Point_2D().from_xy(self.to_xy() + other.to_xy())     # 毕竟得作为xy坐标系才好加嘛

    def __sub__(self, other):
        """
        实现了两个对象的加法运算
        :param other:
        :return: 返回的是一个减好的对象。。。（我真的不知道怎么解释）
        """
        if not isinstance(other, Point_2D):
            raise TypeError("Point_2D can only sub with Point_2D")
        return Point_2D().from_xy(self.to_xy() - other.to_xy())


class Radar_Package(object):
    """
    解析后的数据包
    每个包包括12个点
    """

    rotation_spd = 0  # 转速 deg/s
    start_degree = 0.0  # 扫描开始角度
    points = [Point_2D() for _ in range(12)]  # 12个点的数据
    stop_degree = 0.0  # 扫描结束角度
    time_stamp = 0  # 时间戳 ms 记满30000后重置

    def __init__(self, datas=None):
        if datas is not None:
            self.fill_data(datas)

    def fill_data(self, datas: Tuple[int]):
        self.rotation_spd = datas[0]
        self.start_degree = datas[1] * 0.01
        self.stop_degree = datas[26] * 0.01
        self.time_stamp = datas[27]
        deg_step = (self.stop_degree - self.start_degree) % 360 / 11
        for n, point in enumerate(self.points):
            point.distance = datas[2 + n * 2]
            point.confidence = datas[3 + n * 2]
            point.degree = (self.start_degree + n * deg_step) % 360

    def __str__(self):
        string = (
            f"--- Radar Package < TS = {self.time_stamp:05d} ms > ---\n"
            f"Range: {self.start_degree:06.2f}° -> {self.stop_degree:06.2f}° {self.rotation_spd/360:3.2f}rpm\n"
        )
        for n, point in enumerate(self.points):
            string += f"#{n:02d} {point}\n"
        string += "--- End of Info ---"
        return string

    def __repr__(self):
        return self.__str__()

class Map_360(object):
    """
    将点云数据映射到一个360度的圆上
    每个数据是Point_2D(再次强调)
    包括了很多功能，例如找最近点，但是我觉着这个应该更加区分才行。。。
    有时间第一个就得重构这个
    """

    data = np.ones(360, dtype=np.int64) * -1  # -1: 未知
    time_stamp = np.zeros(360, dtype=np.float64)  # 时间戳
    ######### 映射方法 ########
    MODE_MIN = 0  # 在范围内选择最近的点更新
    MODE_MAX = 1  # 在范围内选择最远的点更新
    MODE_AVG = 2  # 计算平均值更新
    update_mode = MODE_MIN
    ######### 设置 #########
    confidence_threshold = 140  # 置信度阈值
    distance_threshold = 10  # 距离阈值
    timeout_clear = True  # 超时清除
    timeout_time = 1  # 超时时间 s
    ######### 状态 #########
    rotation_spd = 0  # 转速 rpm
    update_count = 0  # 更新计数
    ####### 辅助计算 #######
    _rad_arr = np.deg2rad(np.arange(0, 360))  # 弧度
    _deg_arr = np.arange(0, 360)  # 角度
    _sin_arr = np.sin(_rad_arr)
    _cos_arr = np.cos(_rad_arr)

    def __init__(self):
        pass

    def update(self, data: Radar_Package):
        """
        映射解析后的点云数据
        """
        deg_values_dict = {}
        for point in data.points:
            if (
                point.distance < self.distance_threshold
                or point.confidence < self.confidence_threshold
            ):
                continue
            base = int(point.degree + 0.5)  # 四舍五入
            degs = [base - 1, base, base + 1]  # 扩大点映射范围, 加快更新速度, 降低精度
            # degs = [base] # 只映射实际角度
            for deg in degs:
                deg %= 360
                if deg not in deg_values_dict:
                    deg_values_dict[deg] = set()    # 就当集合处理
                deg_values_dict[deg].add(point.distance)    # 添加扫描点的距离
        # 过滤，三种方法，保证每个方向上只有一个点
        for deg, values in deg_values_dict.items():
            if self.update_mode == self.MODE_MIN:
                self.data[deg] = min(values)
            elif self.update_mode == self.MODE_MAX:
                self.data[deg] = max(values)
            elif self.update_mode == self.MODE_AVG:
                self.data[deg] = int(sum(values) / len(values))
            if self.timeout_clear:
                self.time_stamp[deg] = time.time()
        if self.timeout_clear:
            # 逆天语法
            self.data[self.time_stamp < time.time() - self.timeout_time] = -1
        self.rotation_spd = data.rotation_spd / 360
        self.update_count += 1

    def in_deg(self, from_: int, to_: int) -> List[Point_2D]:
        """
        截取选定角度范围的点
        """
        return [
            Point_2D(deg, self.data[deg])
            for deg in range(from_, to_ + 1)
            if self.data[deg] != -1
        ]

    def clear(self):
        """
        清空数据
        """
        self.data[:] = -1
        self.time_stamp[:] = 0

    def rotation(self, angle: int):
        """
        旋转整个地图, 正角度代表坐标系顺时针旋转, 地图逆时针旋转
        """
        self.data = np.roll(self.data, angle)
        self.time_stamp = np.roll(self.time_stamp, angle)

    def find_nearest(
        self, from_: int = 0, to_: int = 359, num=1, range_limit: int = 1e7, view=None
    ) -> List[Point_2D]:
        """
        在给定范围内查找给定个数的最近点
        from_: int 起始角度
        to_: int 结束角度(包含)
        num: int 查找点的个数
        view: numpy视图, 当指定时上述参数仅num生效
        """
        if view is None:
            view = (self.data < range_limit) & (self.data != -1)
            from_ %= 360
            to_ %= 360
            if from_ > to_:
                view[to_ + 2: from_] = False
            else:
                view[to_ + 2: 360] = False
                view[:from_] = False
        deg_arr = np.where(view)[0]
        data_view = self.data[view]
        p_num = len(deg_arr)
        if p_num == 0:
            return []
        elif p_num <= num:
            sort_view = np.argsort(data_view)
        else:
            sort_view = np.argpartition(data_view, num)[:num]
        points = []
        for index in sort_view:
            points.append(Point_2D(deg_arr[index], data_view[index]))
        return points

    def find_nearest_with_ext_point_opt(
        self, from_: int = 0, to_: int = 359, num=1, range_limit: int = 1e7
    ) -> List[Point_2D]:
        """
        在给定范围内查找给定个数的最近点, 只查找极值点
        from_: int 起始角度
        to_: int 结束角度(包含)
        num: int 查找点的个数
        range_limit: int 距离限制
        """
        view = (self.data < range_limit) & (self.data != -1)
        from_ %= 360
        to_ %= 360
        if from_ > to_:
            view[to_ + 2: from_] = False
        else:
            view[to_ + 2: 360] = False
            view[:from_] = False
        data_view = self.data[view]
        deg_arr = np.where(view)[0]
        peak = find_peaks(-data_view)[0]
        if len(data_view) > 2:
            if data_view[-1] < data_view[-2]:
                peak = np.append(peak, len(data_view) - 1)
        peak_deg = deg_arr[peak]
        new_view = np.zeros(360, dtype=bool)
        new_view[peak_deg] = True
        return self.find_nearest(from_, to_, num, range_limit, new_view)

    def find_two_point_with_given_distance(
        self,
        from_: int,
        to_: int,
        distance: int,
        range_limit: int = 1e7,
        threshold: int = 15,
    ) -> List[Point_2D]:
        """
        在给定范围内查找两个给定距离的点
        from_: int 起始角度
        to_: int 结束角度(包含)
        distance: int 给定的两点之间的距离
        range_limit: int 距离限制
        threshold: int 允许的距离误差
        """
        fd_points = self.find_nearest(from_, to_, 20, range_limit)
        num = len(fd_points)
        get_list = []
        if num >= 2:
            for i in range(num):
                for j in range(i + 1, num):
                    vector = fd_points[i].to_xy() - fd_points[j].to_xy()
                    delta_dis = np.sqrt(vector.dot(vector))
                    if abs(delta_dis - distance) < threshold:
                        deg = (
                            abs(
                                fd_points[i].to_180_degree()
                                + fd_points[j].to_180_degree()
                            )
                            / 2
                        )
                        dis = (fd_points[i].distance +
                               fd_points[j].distance) / 2
                        get_list.append((fd_points[i], fd_points[j], dis, deg))
            if len(get_list) > 0:
                get_list.sort(key=lambda x: x[2])  # 按距离排序
                get_list.sort(key=lambda x: x[3])  # 按角度排序
                return list(get_list[0][:2])
        return []

    def draw_on_cv_image(
        self,
        img: np.ndarray,
        scale: float = 1,
        color: tuple = (0, 0, 255),
        point_size: int = 1,
        add_points: List[Point_2D] = [],
        add_points_color: tuple = (0, 255, 255),
        add_points_size: int = 1,
    ):
        img_size = img.shape
        center_point = np.array([img_size[1] / 2, img_size[0] / 2])
        points_pos = (
            np.array(
                [
                    self.data * self._sin_arr,
                    -self.data * self._cos_arr,
                ]
            )
            * scale
        )
        for n in range(360):
            pos = points_pos[:, n] + center_point
            if self.data[n] != -1:
                cv2.circle(img, tuple(pos.astype(int)), point_size, color, -1)
        for point in add_points:
            pos = center_point + point.to_cv_xy() * scale
            cv2.circle(
                img, (int(pos[0]), int(pos[1])
                      ), add_points_size, add_points_color, -1
            )
        cv2.putText(
            img,
            f"RPM={self.rotation_spd:.2f} CNT={self.update_count}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
        )
        return img

    def output_cloud(self, scale: float = 0.1, size=800) -> np.ndarray:
        black_img = np.zeros((size, size, 1), dtype=np.uint8)
        center_point = np.array([size // 2, size // 2])
        points_pos = (
            np.array(
                [
                    self.data * self._sin_arr,
                    -self.data * self._cos_arr,
                ]
            )
            * scale
        )
        for n in range(360):
            pos = points_pos[:, n] + center_point
            if self.data[n] != -1:
                if 0 <= pos[0] < size and 0 <= pos[1] < size:
                    black_img[int(pos[1]), int(pos[0])] = 255
        return black_img

    def get_distance(self, angle: int) -> int:
        return self.data[int(angle % 360)]

    def get_point(self, angle: int) -> Point_2D:
        return Point_2D(angle, self.data[int(angle % 360)])

    # 对象描述函数，作用：返回当前对象的字符串类型的信息描述，一般用于对象的直接输出显示。
    def __str__(self):
        string = "--- 360 Degree Map ---\n"
        invalid_count = 0
        for deg in range(360):
            if self.data[deg] == -1:
                invalid_count += 1
                continue
            string += f"{deg:03d}° = {self.data[deg]} mm\n"
        if invalid_count > 0:
            string += f"Hided {invalid_count:03d} invalid points\n"
        string += "--- End of Info ---"
        return string

    def __repr__(self):
        return self.__str__()


