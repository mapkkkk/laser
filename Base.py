# coding=UTF-8
import threading
import time
import traceback

from classSerial import FC_Serial
from classData import FC_State_Struct, FC_Settings_Struct
from classLogger import logger

'''
发送类
用来实例化作为接收发送端
2022.11.20
这里的logger完全照搬（搞得好想其他不是照搬的一样）
并且完全没有搞懂logger是干啥用的，大概是python的控制台组件。。。
'''


class class_communicator(object):

    def __init__(self):
        super().__init__()
        self.running = False
        # 初始位
        # 注意，一定要用数组，在bytes函数要求中就是如此
        self._start_bit = [0xAA]
        # 实例化串口
        self._ser_32 = None
        # 设置option的初始值
        self._set_option(0)
        self.running = False
        self.connected = False
        self._thread_list = []
        self._print_state_flag = False
        self._state_update_callback = None
        self.state = FC_State_Struct()
        self.settings = FC_Settings_Struct()
        self._waiting_ack = False
        self._received_ack = None
        self._send_lock = threading.Lock()
        # 开始监听飞控发来的东西
        self.start_listen_serial('/dev/ttyAMA0', 500000, False)

    # option的设置函数
    def _set_option(self, option: int):
        self._ser_32.send_config(startBit=self._start_bit, optionBit=[option])

    # 接收子线程初始化
    # 在发送之前必须先设置这个监听，不然串口不会被定义
    def start_listen_serial(
            self,
            serial_port: str,
            bit_rate: int = 500000,
            print_state=True,
            callback=None,
    ):
        self._state_update_callback = callback
        self._print_state_flag = print_state

        self._ser_32 = FC_Serial(serial_port, bit_rate)

        self._set_option(0)
        self._ser_32.read_config(startBit=[0xAA])
        logger.info("[FC] Serial port opened")
        self.running = True
        _listen_thread = threading.Thread(target=self._listen_serial_task)
        _listen_thread.daemon = True  # 后台驻留程序标签
        _listen_thread.start()
        self._thread_list.append(_listen_thread)

    # 后台任务，子线程
    def _listen_serial_task(self):
        logger.info("[FC] listen serial thread started")
        last_heartbeat_time = time.time()
        while self.running:
            try:
                if self._ser_32.read():  # 读进
                    _data = self._ser_32.rx_data
                    # logger.debug(f"[FC] Read: {bytes_to_str(_data)}")
                    cmd = _data[0]
                    data = _data[1:]
                    if cmd == 0x01:  # 状态回传
                        self._update_state(data)
                    elif cmd == 0x02:  # ACK返回
                        self._received_ack = data[0]
                        self._waiting_ack = False
            except Exception:
                logger.error(f"[FC] listen serial exception: {traceback.format_exc()}")
            if time.time() - last_heartbeat_time > 0.25:
                self.send_data_to_fc(b"\x01", 0x00)  # 心跳包
                last_heartbeat_time = time.time()
            time.sleep(0.001)  # 降低CPU占用

    # 标注一下：这里的发送还是得调用SerialClass里的FC_serial
    # 调用serial发送，发送函数
    def send_data_to_fc(self, data: bytes, option: int, need_ack: bool = False, _ack_retry_count: int = None):
        """将数据向飞控发送, 并等待应答, 一切操作都将由该函数发送, 因此重构到
                其他通讯方式时只需重构该函数即可

                Args:
                    data (bytes): bytes类型的数据
                    option (int): 选项, 对应飞控代码
                    need_ack (bool, optional): 是否需要应答验证. Defaults to False.
                    _ack_retry_count (int, optional): 应答超时时最大重发次数, 此处由函数自动递归设置, 请修改settings中的选项.

                Returns:
                    bytes: 实际发送的数据帧
                """

        if need_ack:
            if _ack_retry_count is None:
                _ack_retry_count = self.settings.ack_max_retry
            if _ack_retry_count < 0:
                # raise Exception("Wait ACK reached max retry")
                logger.error("Wait ACK reached max retry")
                return None
            self._waiting_ack = True
            self._received_ack = None
            send_time = time.time()
            check_ack = option
            for add_bit in data:
                check_ack = (check_ack + add_bit) & 0xFF

        try:
            self._send_lock.acquire(timeout=self.settings.wait_sending_timeout)
        except:
            logger.error("[FC] Wait sending data timeout")
            return None

        self._set_option(option)
        sent = self._ser_32.write(data)
        self._send_lock.release()

        if need_ack:
            while self._waiting_ack:
                if time.time() - send_time > self.settings.wait_ack_timeout:
                    logger.warning("[FC] ACK timeout, retrying")
                    return self.send_data_to_fc(
                        data, option, need_ack, _ack_retry_count - 1
                    )
                time.sleep(0.001)
            if self._received_ack is None or self._received_ack != check_ack:
                logger.warning("[FC] ACK not received or invalid, retrying")
                return self.send_data_to_fc(
                    data, option, need_ack, _ack_retry_count - 1
                )
        return sent

    # 退出函数
    def quit(self):
        self.running = False
        if self._ser_32:
            self._ser_32.close()

    # 更新飞机状态
    def _update_state(self, recv_byte):
        try:
            # index = 0
            # for var in self.state.RECV_ORDER:
            #     length = var.byte_length
            #     var.bytes = recv_byte[index : index + length]
            #     index += length
            self.state.update_from_bytes(recv_byte)
            if not self.connected:
                self.connected = True
                logger.info("[FC] Connected")
            if callable(self._state_update_callback):
                self._state_update_callback(self.state)
            if self._print_state_flag:
                self._print_state()
        except Exception:
            logger.error(f"[FC] Update state exception: {traceback.format_exc()}")

    def _print_state(self):
        color_red = "\033[1;31m"
        color_green = "\033[1;32m"
        YELLOW = "\033[1;33m"
        BLUE = "\033[1;34m"
        CYAN = "\033[1;36m"
        PURPLE = "\033[1;35m"
        RESET = "\033[0m"
        text = ""
        text += " ".join(
            [
                f"{YELLOW}{(var.name[0] + var.name[-1])}: "
                f"{f'{color_green}√ ' if var.value else f'{color_red}x {RESET}'}"
                if type(var.value) == bool
                else (
                    f"{YELLOW}{(var.name[0] + var.name[-1])}:{CYAN}{var.value:^7.02f}{RESET}"
                    if type(var.value) == float
                    else f"{YELLOW}{(var.name[0] + var.name[-1])}:{CYAN}{var.value:^4d}{RESET}"
                )
                for var in self.state.RECV_ORDER
            ]
        )
        print(f"\r {text}\r", end="")
