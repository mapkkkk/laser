# coding=UTF-8
from time import sleep
from classData import Byte_Var
from classData import class_option

'''
基本任务结构体
实例化后直接调用
学长管这个叫protocol，感觉挺正确但是懒得改
HZW
'''


class class_mission:
    sendclass = None
    option = None
    # 数据定义
    unlock_data = Byte_Var("u8", int, 1)
    unlock_data.value = 0x01
    lock_data = Byte_Var("u8", int, 1)
    lock_data.value = 0x02
    takeoff_height = Byte_Var("u16", int, 1)
    takeoff_height.value = 0
    land_data = Byte_Var("u8", int, 1)
    land_data.value = 0x01
    first_mode_data = Byte_Var("u8", int, 1)
    first_mode_data.value = 0x01
    second_mode_data = Byte_Var("u8", int, 1)
    second_mode_data.value = 0x02
    program_dis = Byte_Var("u16", int, 1)
    program_dis.value = 0
    program_vel = Byte_Var("u16", int, 1)
    program_vel.value = 0
    program_angle = Byte_Var("u16", int, 1)
    program_angle.value = 0
    rc_vel_x = Byte_Var("s16", int, 1)
    rc_vel_x.value = 0
    rc_vel_y = Byte_Var("s16", int, 1)
    rc_vel_y.value = 0
    rc_vel_z = Byte_Var("s16", int, 1)
    rc_vel_z.value = 0
    rc_yaw = Byte_Var("s16", int, 1)
    rc_yaw.value = 0
    start_beep_data = Byte_Var("u8", int, 1)
    start_beep_data.value = 0x01
    stop_beep_data = Byte_Var("u8", int, 1)
    stop_beep_data.value = 0x02

    def __init__(self, sendclass=None):
        self.sendclass = sendclass
        self.option = class_option()

    def takeoff(self, height):
        # 解锁
        print(self.unlock_data.bytes)
        self.takeoff_height.value = height
        self.sendclass.send_data_to_fc(self.unlock_data.bytes, self.option.lock_unlock)
        sleep(2)
        # 切换到程控模式起飞
        self.sendclass.send_data_to_fc(self.second_mode_data.bytes, self.option.mode_change)
        sleep(0.2)
        # 起飞
        self.sendclass.send_data_to_fc(self.takeoff_height.bytes, self.option.take_off)
        sleep(3)
        # 返回1作为执行完成标记
        return 1

    def land(self):
        # 切换到程控
        self.sendclass.send_data_to_fc(self.second_mode_data.bytes, self.option.mode_change)
        sleep(0.2)
        # 降落
        self.sendclass.send_data_to_fc(self.land_data.bytes, self.option.land)
        sleep(3)
        return 1

    def program_control_move(self, dis, vel, angle):
        self.program_dis.value = dis
        self.program_vel.value = vel
        self.program_angle.value = angle
        data_to_send = self.program_dis.bytes + self.program_vel.bytes + self.program_angle.bytes
        self.sendclass.send_data_to_fc(data_to_send, self.option.program_control)

    def realtime_control_send(self):
        data_to_send = self.rc_vel_x.bytes + self.rc_vel_y.bytes + self.rc_vel_z.bytes + self.rc_yaw.bytes
        self.sendclass.send_data_to_fc(data_to_send, self.option.realtime_control)

    def realtime_control_config(self, vel_x, vel_y, vel_z, yaw):
        self.rc_vel_x.value = vel_x
        self.rc_vel_y.value = vel_y
        self.rc_vel_z.value = vel_z
        self.rc_yaw.value = yaw

    def realtime_control_reset(self):
        self.rc_vel_x.value = 0
        self.rc_vel_y.value = 0
        self.rc_vel_z.value = 0
        self.rc_yaw.value = 0

    def mode_set(self, mode):
        if mode == 1:
            self.sendclass.send_data_to_fc(self.first_mode_data.bytes, self.option.mode_change)
        elif mode == 2:
            self.sendclass.send_data_to_fc(self.second_mode_data.bytes, self.option.mode_change)

    def start_beep(self):
        self.sendclass.send_data_to_fc(self.start_beep_data.bytes, self.option.beep)

    def stop_beep(self):
        self.sendclass.send_data_to_fc(self.stop_beep_data.bytes, self.option.beep)
