from copy import copy
from sys import byteorder as sysByteorder

import serial


# 这一块一会也得改不少东西
# 2022.10.6这一块基本改完了（虽然就是注释了接收的代码）
class FC_Serial:

    def __init__(self, port, baudrate, timeout=0.5, byteOrder=sysByteorder):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)  # 就读0.5ms
        self.read_buffer = bytes()  # 读数组
        self.read_save_buffer = bytes()  # 这是啥
        self.reading_flag = False  # 某个标志
        # self.read_save_buffer = bytes()  # 逆天，重复定义？
        self.pack_count = 0  # 包的个数？
        self.pack_length = 0  # 包的长度？
        self.byte_order = byteOrder  # 字符顺序，具体干啥用的还没想通
        self.waiting_buffer = bytes()  # 啥？似乎是某个缓存区
        self.send_config()  # 发送的输入参数的函数
        self.read_config()  # 接收的输入参数的函数

    def send_config(self, startBit=[], optionBit=[]):
        self.send_start_bit = startBit
        self.send_option_bit = optionBit

    # def read_config(self, startBit=[]):
    #     self.read_start_bit = startBit

    # 校验函数
    # def check_rx_data_sum(self):
    #     length = len(self.read_buffer)
    #     checksum = 0
    #     for i in self.read_start_bit:
    #         checksum += i
    #         checksum &= 0xFF
    #     checksum += self.pack_length_bit
    #     checksum &= 0xFF
    #     for i in range(0, length):
    #         checksum += int.from_bytes(
    #             self.read_buffer[i: i + 1],
    #             byteorder=self.byte_order,
    #             signed=False,
    #         )
    #         checksum &= 0xFF
    #     received_checksum = int.from_bytes(
    #         self.ser.read(1), byteorder=self.byte_order, signed=False
    #     )
    #     if received_checksum == checksum:
    #         return 1
    #     return 0

    # 读函数（方法）
    # def read(self) -> bool:
    #     tmp = 0x00
    #     while self.ser.in_waiting > 0:
    #         tmp = self.ser.read(1)  # Read size bytes from the serial port. If a timeout is set it may
    #         _len = len(self.read_start_bit)
    #         if not self.reading_flag:
    #             self.waiting_buffer += tmp
    #             if len(self.waiting_buffer) >= _len:
    #                 if self.waiting_buffer[-_len:] == bytes(self.read_start_bit):
    #                     self.reading_flag = True
    #                     self.read_buffer = bytes()
    #                     self.pack_count = 0
    #                     self.pack_length = -1
    #                     self.waiting_buffer = bytes()
    #             continue
    #         if self.pack_length == -1:
    #             self.pack_length_bit = int.from_bytes(
    #                 tmp, self.byte_order, signed=False
    #             )
    #             self.pack_length = self.pack_length_bit & 0b11111111
    #             continue
    #         if self.reading_flag:
    #             self.pack_count += 1
    #             self.read_buffer += tmp
    #             if self.pack_count >= self.pack_length:
    #                 self.reading_flag = False
    #                 if self.check_rx_data_sum():
    #                     self.read_save_buffer = copy(self.read_buffer)
    #                     self.read_buffer = bytes()
    #                     return True
    #                 else:
    #                     self.read_buffer = bytes()
    #                     return False
    #     return False
    #
    # @property
    # def rx_data(self):
    #     return self.read_save_buffer

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
                bytes(self.sned_start_bit)
                + bytes(self.send_option_bit)
                + len_as_byte
                + data
        )

        # checksum = 0
        # for i in range(0, len(send_data)):
        #     checksum += send_data[i]
        #     checksum &= 0xFF
        # send_data += checksum.to_bytes(1, self.byte_order)

        self.ser.write(send_data)
        self.ser.flush()
        return send_data
