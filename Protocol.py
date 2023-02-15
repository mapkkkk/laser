# coding=UTF-8
import struct
from time import sleep

from classData import class_option
from classCommunicator import class_communicator
from classData import Data_To_FC

'''
基本任务结构体
实例化后直接调用
学长管这个叫protocol，感觉挺正确但是懒得改
结果还是改了
HZW
'''


class class_protocol(class_communicator):
    option = None
    # 数据定义
    data_to_fc = None
    HOLD_ALT_MODE = 1
    HOLD_POS_MODE = 2
    PROGRAM_MODE = 3

    def __init__(self):
        super().__init__()
        '''
        在上一个类中已经完成底层通讯的搭建，send和receive都可直接调用，且能自动化运行
        并且，（如果我没理解错的话），监听也启动了，并且一直在后台工作
        在这个类中完成一部分的"一键"操作
        '''
        self.option = class_option()
        self.data_to_fc = Data_To_FC()

    def takeoff(self, height):
        # 解锁
        # print(self.data_to_fc.unlock_data.bytes)
        self.data_to_fc.takeoff_height.value = height
        self.send_data_to_fc(self.data_to_fc.unlock_data.bytes, self.option.lock_unlock)
        sleep(2)
        # 切换到程控模式起飞
        self.send_data_to_fc(self.data_to_fc.second_mode_data.bytes, self.option.mode_change)
        sleep(0.2)
        # 起飞
        self.send_data_to_fc(self.data_to_fc.takeoff_height.bytes, self.option.takeoff)
        sleep(3)
        # 返回1作为执行完成标记
        return 1

    def land(self):
        # 切换到程控
        self.send_data_to_fc(self.data_to_fc.second_mode_data.bytes, self.option.mode_change)
        sleep(0.2)
        # 降落
        self.send_data_to_fc(self.data_to_fc.land_data.bytes, self.option.land)
        sleep(3)
        return 1

    def program_control_move(self, dis, vel, angle):
        self.data_to_fc.program_dis.value = dis
        self.data_to_fc.program_vel.value = vel
        self.data_to_fc.program_angle.value = angle

        data_to_send = (self.data_to_fc.program_dis.bytes +
                        self.data_to_fc.program_vel.bytes +
                        self.data_to_fc.program_angle.bytes)

        self.send_data_to_fc(data_to_send, self.option.program_control)

    # def realtime_control_send(self):
    #     data_to_send = self.data_to_fc.rc_vel_x.bytes + \
    #                    self.data_to_fc.rc_vel_y.bytes + \
    #                    self.data_to_fc.rc_vel_z.bytes + \
    #                    self.data_to_fc.rc_yaw.bytes
    #
    #     self.send_data_to_fc(data_to_send, self.option.realtime_control)

    def realtime_control_send(self, vel_x: int = None, vel_y: int = None, vel_z: int = None, yaw: int = 0):
        data = struct.pack("<hhhh", int(vel_x), int(vel_y), int(vel_z), int(-yaw))  # 这里的<指的是小端，h是short int
        self.send_data_to_fc(data, self.option.realtime_control, False)

    def realtime_control_reset(self):
        data = struct.pack("<hhhh", 0, 0, 0, 0)
        self.send_data_to_fc(data, self.option.realtime_control, False)

    def mode_set(self, mode):
        if mode == 1:
            self.send_data_to_fc(self.data_to_fc.first_mode_data.bytes, self.option.mode_change)
        elif mode == 2:
            self.send_data_to_fc(self.data_to_fc.second_mode_data.bytes, self.option.mode_change)

    def start_beep(self):
        self.send_data_to_fc(self.data_to_fc.start_beep_data.bytes, self.option.beep)

    def stop_beep(self):
        self.send_data_to_fc(self.data_to_fc.stop_beep_data.bytes, self.option.beep)
