# coding=UTF-8
import struct
import time
from ProtocolMCU.Data import class_option
from ProtocolMCU.Base import base_communicate
from ProtocolMCU.Data import Data_To_FC
from ProtocolMCU.Data import Byte_Var
from others.Logger import logger
'''
基本任务底层
HZW
NOTE:
ALL DONE
'''


class class_protocol(base_communicate):
    # 数据定义
    HOLD_ALT_MODE = 1
    HOLD_POS_MODE = 2
    PROGRAM_MODE = 3

    def __init__(self) -> None:
        super().__init__()
        self.byte_temp1 = Byte_Var('u8', int)
        self.byte_temp2 = Byte_Var('u8', int)
        self.byte_temp3 = Byte_Var('u8', int)
        '''
        在上一个类中已经完成底层通讯的搭建,send和receive都可直接调用,且能自动化运行
        在这个类中完成一部分的"一键"操作
        '''
        self.option = class_option()
        self.data_to_fc = Data_To_FC()

    '''
    TODO:
    1、更改数据通信格式:帧头+mainoption(标记是主要命令还是转发的cmd)+datalen+suboption+data+checksum
    帧头、mainoption、datalen、checksum均在serial中完成添加,此处仅需考虑中间data部分
    关于suboption可以认为仅在此处有使用到,这边一会mark一下所有的分别是什么
    
    '''

    def check_mode(self, target_mode) -> bool:
        """
        检查当前模式是否与需要的模式一致
        """
        mode_dict = {1: "HOLD ALT", 2: "HOLD POS", 3: "PROGRAM"}
        if self.state.mode.value != target_mode:
            if self.settings.auto_change_mode:
                self.set_flight_mode(target_mode)
                time.sleep(0.1)  # 等待模式改变完成
                return True
            else:
                logger.error(
                    f"[FC] Mode error: action required mode is {mode_dict[target_mode]}"
                    f", but current mode is {mode_dict[self.state.mode.value]}"
                )
                return False
        return True

# 命令发送，经过MCU处理进行控制

    def send_command(self,
                     sub_option: int,
                     data: bytes = b"",
                     need_ack=False) -> None:
        """
        :param sub_option: 填入子选项
        :param data: 数据
        :param need_ack: 是否需要检查
        :return:
        """

        self.byte_temp1.reset(sub_option, "u8", int)
        self.send_data_to_fc(self.byte_temp1.bytes + data,
                             0x01,
                             need_ack=need_ack)

    def send_realtime_control_data(self,
                                   vel_x: int = 0,
                                   vel_y: int = 0,
                                   vel_z: int = 0,
                                   yaw: int = 0) -> None:
        """
        发送实时控制帧, 仅在定点模式下有效(MODE=2), 切换模式前需要确保遥控器摇杆全部归中
        有熔断机制，超过一秒的帧间隔会导致熔断，再次获得数据后会恢复
        在application中完成实时控制的完全体
        vel_x,vel_y,vel_z: cm/s 匿名坐标系
        yaw: deg/s 顺时针为正
        """
        data = struct.pack("<hhhh", int(vel_x), int(vel_y), int(vel_z),
                           int(-yaw))
        self.send_command(0x01, data)  # 帧结尾

    def start_beep(self):
        """
        蜂鸣器控制
        """
        self.byte_temp1.reset(0x01, "u8", int)
        self.send_command(0x02, self.byte_temp1.bytes, need_ack=True)

    def stop_beep(self):
        """
        蜂鸣器控制
        """
        self.byte_temp1.reset(0x02, "u8", int)
        self.send_command(0x02, self.byte_temp1.bytes, need_ack=False)

    def set_pwm_output(self, channel: int, pwm: float) -> None:
        """
        设置PWM输出
        channel: 0-3
        pwm: 0.00-100.00
        """
        assert channel in [0, 1, 2, 3]
        pwm_int = int(pwm * 100)
        pwm_int = max(0, min(10000, pwm_int))
        self.byte_temp1.reset(channel, "u8", int)
        self.byte_temp2.reset(pwm_int, "s16", int)
        self.send_command(
            0x04,
            self.byte_temp1.bytes + self.byte_temp2.bytes,
            True,  # need ack
        )

# IMU直接转发

    def _send_imu_command_frame(self,
                                CID: int,
                                CMD0: int,
                                CMD1: int,
                                CMD_data=b""):
        self.byte_temp1.reset(CID, "u8", int)
        self.byte_temp2.reset(CMD0, "u8", int)
        self.byte_temp3.reset(CMD1, "u8", int)
        bytes_data = bytes(CMD_data)
        if len(bytes_data) < 8:
            bytes_data += b"\x00" * (8 - len(bytes_data))
        if len(bytes_data) > 8:
            raise Exception("CMD_data length is too long")
        data_to_send = (self.byte_temp1.bytes + self.byte_temp2.bytes +
                        self.byte_temp3.bytes + bytes_data)
        self.send_data_to_fc(data_to_send, 0x02, need_ack=True)
        # cid = 0x10, cmd0 = 0x00 cmd1 = 0x04 -> hovering
        self.last_sended_command = (CID, CMD0, CMD1)

    def set_flight_mode(self, mode: int) -> None:
        """
        设置飞行模式: (随时有效)
        0: 姿态自稳 (危险,禁用)
        1: 定高
        2: 定点
        3: 程控
        """
        if mode not in [1, 2, 3]:
            raise ValueError("mode must be 1,2,3")
        self.byte_temp1.reset(mode, "u8", int)
        self._send_imu_command_frame(0x01, 0x01, 0x01, self.byte_temp1.bytes)

    def unlock(self):
        """
        解锁
        """
        self._send_imu_command_frame(0x10, 0x00, 0x01)

    def lock(self):
        """
        上锁
        """
        self._send_imu_command_frame(0x10, 0x00, 0x02)

    def takeoff(self, target_height):
        """
        一键起飞(除姿态模式外, 随时有效)
        目标高度: 0-500 cm, 0为默认高度
        """
        self.byte_temp1.reset(target_height, "u16", int)
        self._send_imu_command_frame(0x10, 0x00, 0x05, self.byte_temp1.bytes)

    def land(self):
        """
        一键降落(除姿态模式外, 随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x06)

    def stabilize(self):
        """
        恢复定点悬停, 将终止正在进行的所有控制(随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x04)

    def horizontal_move(self, distance: int, speed: int,
                        direction: int) -> None:
        """
        水平移动: (程控模式下有效)
        移动距离:0-10000 cm
        移动速度:10-300 cm/s
        移动方向:0-359 度 (当前机头为0参考,顺时针)
        """
        self.check_mode(3)
        self.byte_temp1.reset(distance, "u16", int)
        self.byte_temp2.reset(speed, "u16", int)
        self.byte_temp3.reset(direction, "u16", int)
        self._send_imu_command_frame(
            0x10,
            0x02,
            0x03,
            self.byte_temp1.bytes + self.byte_temp2.bytes +
            self.byte_temp3.bytes,
        )

    def go_up(self, distance: int, speed: int) -> None:
        """
        上升: (程控模式下有效)
        上升距离:0-10000 cm
        上升速度:10-300 cm/s
        """
        self.check_mode(3)
        self.byte_temp1.reset(distance, "u16", int)
        self.byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(
            0x10, 0x02, 0x01, self.byte_temp1.bytes + self.byte_temp2.bytes)

    def go_down(self, distance: int, speed: int) -> None:
        """
        下降: (程控模式下有效)
        下降距离:0-10000 cm
        下降速度:10-300 cm/s
        """
        self.check_mode(3)
        self.byte_temp1.reset(distance, "u16", int)
        self.byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(
            0x10, 0x02, 0x02, self.byte_temp1.bytes + self.byte_temp2.bytes)

    def turn_left(self, deg: int, speed: int) -> None:
        """
        左转: (程控模式下有效)
        左转角度:0-359 度
        左转速度:5-90 deg/s
        """
        self.check_mode(3)
        self.byte_temp1.reset(deg, "u16", int)
        self.byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(
            0x10, 0x02, 0x07, self.byte_temp1.bytes + self.byte_temp2.bytes)

    def turn_right(self, deg: int, speed: int) -> None:
        """
        右转: (程控模式下有效)
        右转角度:0-359 度
        右转速度:5-90 deg/s
        """
        self.check_mode(3)
        self.byte_temp1.reset(deg, "u16", int)
        self.byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(
            0x10, 0x02, 0x08, self.byte_temp1.bytes + self.byte_temp2.bytes)

    def set_yaw(self, yaw: int, speed: int) -> None:
        """
        设置偏航角: (程控模式下有效)
        偏航角:-180-180 度
        偏航速度:5-90 deg/s
        """
        # self._action_log("set yaw", f"{yaw}deg, {speed}deg/s")
        current_yaw = self.state.yaw.value
        if yaw < current_yaw:
            left_turn_deg = abs(current_yaw - yaw)
            right_turn_deg = abs(360 - left_turn_deg)
        else:
            right_turn_deg = abs(current_yaw - yaw)
            left_turn_deg = abs(360 - right_turn_deg)
        if left_turn_deg < right_turn_deg:
            self.turn_left(left_turn_deg, speed)
        else:
            self.turn_right(right_turn_deg, speed)

    @property
    def last_command_done(self) -> bool:
        """
        最后一次指令是否完成
        """
        return self.last_sended_command != self.state.command_now

    @property
    def hovering(self) -> bool:
        """
        是否正在悬停
        """
        stable_command = (0x10, 0x00, 0x04)
        return self.state.command_now == stable_command
