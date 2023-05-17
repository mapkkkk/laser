# coding=UTF-8
from copy import copy
from sys import byteorder as sys_byteorder
import serial

'''
这一块一会也得改不少东西
2022.10.6这一块基本改完了（虽然就是注释了接收的代码）
2022.11.1(现在这个接收代码得完全重新写了)(因为我这个sb把它全删了)
2022.11.6 在复变考完之后开始搞双端通信
python的串口开启有定时, 如果一直开启会发生啥?
从飞控往会传的数据大概有飞控状态、少量传感器数据、后面可能需要加上的校验，还有啥就暂时不知道了
这里的类要写成通用的，因为要满足很多的情况下使用
传入的start_bit一定要是列表形式哇, 别忘了
串口类
为发送接收类服务
'''


class FC_Serial:

    def __init__(self, port, baudrate, timeout=0.5, byteOrder=sys_byteorder):
        self.send_start_bit = [0xAA]
        self.send_option_bit = [0x00]
        self.read_start_bit = [0xAA]
        self.read_flag = False
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.read_buffer = bytes()
        self.read_save_buffer = bytes()
        self.waiting_buffer = bytes()
        self.byte_order = byteOrder
        self.frame_count = 0
        self.frame_length = 0
        self.frame_length_bit = 0
        # self.send_config()
        # self.read_config()

    def send_config(self, startBit: list, optionBit: list):
        self.send_start_bit = startBit
        self.send_option_bit = optionBit

    def read_config(self, startBit: list):
        self.read_start_bit = startBit

    def read_one_bit(self):
        while self.ser.in_waiting > 0:
            # 读一个字节
            tmp = self.ser.read(1)
            # print(tmp)
            start_bit_len = len(self.read_start_bit)
            if not self.read_flag:
                self.waiting_buffer += tmp
                if len(self.waiting_buffer) > start_bit_len:
                    # 为了通用性考虑了开始位不止一位，在waiting_buffer中的末位考虑
                    # 但是这个waiting_buffer到底是干啥用的真搞不太清楚
                    if self.waiting_buffer[-start_bit_len:] == bytes(
                            self.read_start_bit):
                        self.read_flag = True
                        self.read_buffer = bytes()
                        self.frame_count = 0
                        self.frame_length = -1
                        self.waiting_buffer = bytes()
                continue

            if self.frame_length == -1:
                self.frame_length_bit = int.from_bytes(tmp,
                                                       self.byte_order,
                                                       signed=False)
                self.frame_length = self.frame_length_bit & 0b11111111  # 为激光雷达预留
                # print(self.frame_length)
                # self.frame_length += 1
                continue

            if self.read_flag:
                self.frame_count += 1
                self.read_buffer += tmp
                if self.frame_count >= self.frame_length:
                    self.read_flag = False
                    if self.check_rx_data_sum():
                        self.read_save_buffer = copy(self.read_buffer)
                        self.read_buffer = bytes()
                        return 1
                    else:
                        # print(self.read_buffer)
                        self.read_buffer = bytes()
                        # print(False)
                        return 0
            return 0

    def check_rx_data_sum(self):
        length = len(self.read_buffer)
        # print(self.read_buffer)
        checksum = 0
        for i in self.read_start_bit:
            checksum += i
            checksum &= 0xFF
        checksum += self.frame_length_bit
        checksum &= 0xFF
        for i in range(0, length):
            checksum += int.from_bytes(
                self.read_buffer[i:i + 1],
                byteorder=self.byte_order,
                signed=False,
            )
            checksum &= 0xFF
        received_checksum = int.from_bytes(self.ser.read(1),
                                           byteorder=self.byte_order,
                                           signed=False)
        # print(received_checksum, checksum)
        if received_checksum == checksum:
            return True
        return False

    def write(self, data: bytes):
        """
        The function takes a byte array as input, and then sends it to the serial port

        :param data: The data to be sent
        :type data: bytes
        :return: The data that was sent.
        """
        data = copy(data)
        if isinstance(data, list):
            data = bytes(data)
        if not isinstance(data, bytes):
            raise TypeError("data must be bytes")
        len_as_byte = len(data).to_bytes(1, self.byte_order)
        send_data = (bytes(self.send_start_bit) + bytes(self.send_option_bit) +
                     len_as_byte + data)
        # 校验
        checksum = 0
        for i in range(0, len(send_data)):
            checksum += send_data[i]
            checksum &= 0xFF  # 只取低八位
        send_data += checksum.to_bytes(1, self.byte_order)
        self.ser.write(send_data)
        self.ser.flush()
        # print(send_data)
        return send_data

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    @property
    def rx_data(self):
        return self.read_save_buffer
