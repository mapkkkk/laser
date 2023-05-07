import numpy as np
import cv2
from scipy.signal import find_peaks
from typing import List, Literal
from RadarDrivers_reconstruct.RadarMapBase import Point_2D
from RadarDrivers_reconstruct.RadarSerialUpdater import radar_serial_updater


def get_point_line_distance(
        x1, y1, x2, y2, x3, y3, line_type: Literal[0, 1] = 0
) -> tuple[float, float]:
    """
    计算点到线的距离
    p1: 目标点
    p2: 线的一个端点
    p3: 线的另一个端点
    type: 0: 右侧线, 1: 下侧线
    return: 距离, 角度
    """

    theta = 0

    def deg_180_90(deg):
        if deg > 90:
            deg = deg - 180
        return deg

    distance = abs((y3 - y2) * x1 - (x3 - x2) * y1 + x3 * y2 - y3 * x2) / np.sqrt(
        (y3 - y2) ** 2 + (x3 - x2) ** 2
    )
    if line_type == 0:
        theta = deg_180_90(
            (np.pi / 2 + np.arctan((y3 - y2) / (x3 - x2 + 0.0000001))) * 180 / np.pi
        )
    elif line_type == 1:
        theta = np.arctan((y3 - y2) / (x3 - x2 + 0.0000001)) * 180 / np.pi
    return distance, theta


class radar_map_resolve(radar_serial_updater):
    """
    雷达点云图像解析器
    """

    def __init__(self) -> None:
        super().__init__()

    def map_visual_resolve_rt_pose(self, rtpose_size, rtpose_scale_ratio, DEBUG=False):
        """
        从雷达点云图像中解析出中点位置
        img: 雷达点云图像(灰度图)
        _DEBUG: 显示解析结果
        return: 位姿(x,y,yaw)
        """
        # 参数设置
        kernal_di = 9
        kernal_er = 5
        hough_threshold = 80
        min_line_length = 60

        # 二值化
        kernel_di = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernal_di, kernal_di))
        kernel_er = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernal_er, kernal_er))

        # 获取图像
        img = self.output_cloud(
            size=int(rtpose_size),
            scale=0.1 * rtpose_scale_ratio,
        )
        img = cv2.dilate(img, kernel_di)  # 膨胀
        img = cv2.erode(img, kernel_er)  # 腐蚀
        # 霍夫变换
        lines = cv2.HoughLinesP(
            img,
            1,
            np.pi / 180,
            threshold=hough_threshold,
            minLineLength=min_line_length,
            maxLineGap=200,
        )
        size = img.shape
        x0, y0 = size[0] // 2, size[1] // 2

        if DEBUG:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        x_out = None
        y_out = None
        yaw_out_1 = None
        yaw_out_2 = None
        right_lines = []
        back_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if (
                        x1 > x0 and x2 > x0 and (
                        (y1 > y0 > y2) or (y1 < y0 < y2))
                ):  # 右侧线
                    if x1 > x2:
                        right_lines.append((x2, y2, x1, y1))
                    else:
                        right_lines.append((x1, y1, x2, y2))
                elif (
                        y1 > y0 and y2 > y0 and (
                        (x1 > x0 > x2) or (x1 < x0 < x2))
                ):  # 下侧线
                    if y1 > y2:
                        back_lines.append((x2, y2, x1, y1))
                    else:
                        back_lines.append((x1, y1, x2, y2))

        for line in right_lines:
            x1, y1, x2, y2 = line
            dis, yaw = get_point_line_distance(x0, y0, x1, y1, x2, y2, 0)
            if y_out is None or dis < y_out:
                y_out = dis
                yaw_out_1 = -yaw
            if DEBUG:
                cv2.line(img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        for line in back_lines:
            x1, y1, x2, y2 = line
            dis, yaw = get_point_line_distance(x0, y0, x1, y1, x2, y2, 1)
            if x_out is None or dis < x_out:
                x_out = dis
                yaw_out_2 = -yaw
            if DEBUG:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
        if yaw_out_1 and yaw_out_2:
            yaw_out = (yaw_out_1 + yaw_out_2) / 2
        elif yaw_out_1:
            yaw_out = yaw_out_1
        elif yaw_out_2:
            yaw_out = yaw_out_2
        else:
            yaw_out = None

        if DEBUG:
            x_ = x_out if x_out else -1
            y_ = y_out if y_out else -1
            yaw_ = yaw_out if yaw_out else 0
            p = Point_2D(int(-yaw_) + 90, int(y_))
            target = p.to_cv_xy() + np.array([x0, y0])
            cv2.line(img, (x0, y0), (int(target[0]), int(
                target[1])), (0, 0, 255), 1)
            p = Point_2D(int(-yaw_) + 180, int(x_))
            target = p.to_cv_xy() + np.array([x0, y0])
            cv2.line(img, (x0, y0), (int(target[0]), int(
                target[1])), (0, 0, 255), 1)
            cv2.putText(
                img,
                f"({x_:.1f}, {y_:.1f}, {yaw_:.1f})",
                (x0, y0 - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                1,
            )
            cv2.imshow("Map Resolve", img)
        return x_out, y_out, yaw_out

    def find_nearest(
            self, from_: int = 0, to_: int = 359, num=1, range_limit: int = 10000000, view=None
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
            points.append(Point_2D(int(deg_arr[index]), int(data_view[index])))
        return points

    def find_nearest_with_ext_point_opt(
            self, from_: int = 0, to_: int = 359, num=1, range_limit: int = 10000000
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
            range_limit: int = 10000000,
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
