# coding=UTF-8
from classSerial import FC_Serial

'''
发送类
用来实例化作为发送端
'''


class class_communicator:

    def __init__(self, port):
        # super().__init__()
        self.running = False
        # 初始位
        self._start_bit = [0xAA]
        # 实例化串口
        self._ser_32 = FC_Serial(port, 500000)
        # 设置option的初始值
        self._set_option(0)
        # 直接启动，设置状态为启动
        self.running = True

    # option的设置函数
    def _set_option(self, option: int):
        self._ser_32.send_config(startBit=self._start_bit, optionBit=[option])

    # 标注一下：这里的发送还是得调用SerialClass里的FC_serial
    # 调用serial发送，发送函数
    def send_data_to_fc(self, data: bytes, option: int):
        self._set_option(option)
        sent = self._ser_32.write(data)
        return sent

    # 退出函数
    def quit(self, joined=False):
        self.running = False
        if self._ser_32:
            self._ser_32.close()
