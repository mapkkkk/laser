# coding=UTF-8
from copy import copy
from sys import byteorder as sysByteorder

import serial

'''
这一块一会也得改不少东西
2022.10.6这一块基本改完了（虽然就是注释了接收的代码）
串口类
为发送接收类服务
'''


class FC_Serial:

    def __init__(self, port, baudrate, byteOrder=sysByteorder):
        self.send_start_bit = None
        self.send_option_bit = None
        self.ser = serial.Serial(port, baudrate)  # 就读0.5ms
        self.read_buffer = bytes()  # 读数组
        self.read_save_buffer = bytes()  # 这是啥
        self.reading_flag = False  # 某个标志
        self.byte_order = byteOrder  # 字符顺序，具体干啥用的还没想通
        self.waiting_buffer = bytes()  # 啥？似乎是某个缓存区
        self.send_config()  # 发送的输入参数的函数
        # self.read_config()  # 接收的输入参数的函数

    def send_config(self, startBit=[], optionBit=[]):
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
