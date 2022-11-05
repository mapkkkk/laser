# coding=UTF-8
from copy import copy
from sys import byteorder as sys_byteorder

import serial

'''
这一块一会也得改不少东西
2022.10.6这一块基本改完了（虽然就是注释了接收的代码）
2022.11.1（现在这个接收代码得完全重新写了）（因为我这个sb把它全删了）
串口类
为发送接收类服务
'''


class FC_Serial:

    def __init__(self, port, baudrate, byteOrder=sys_byteorder):
        self.send_start_bit = None
        self.send_option_bit = None
        self.ser = serial.Serial(port, baudrate)
        self.read_buffer = bytes()
        self.read_save_buffer = bytes()
        self.reading_flag = False
        self.byte_order = byteOrder
        self.waiting_buffer = bytes()
        # self.send_config()
        # self.read_config()

    def send_config(self, startBit, optionBit):
        self.send_start_bit = startBit
        self.send_option_bit = optionBit

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def write(self, data: bytes):
        data = copy(data)
        if isinstance(data, list):
            data = bytes(data)
        if not isinstance(data, bytes):
            raise TypeError("data must be bytes")
        len_as_byte = len(data).to_bytes(1, self.byte_order)
        send_data = (
                bytes(self.send_start_bit)
                + bytes(self.send_option_bit)
                + len_as_byte
                + data
        )
        self.ser.write(send_data)
        self.ser.flush()
        return send_data
