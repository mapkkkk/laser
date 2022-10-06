import struct
import threading
import time

# 数据结构，很好用了
from SerialClass import FC_Serial


class Byte_Var:
    """
    这部分定义发送数据格式，实现int向s16u16之类的转换
    到头来还是全抄了一遍，反正我自己还没到能手搓这个的境界
    """

    _value = 0
    _last_update_time = 0
    _byte_length = 0
    _multi: int = 1
    _signed = False
    _var_type = None

    def __init__(self, ctype="u8", data_type=int, value_multi=1, name=None):
        self.reset(0, ctype, data_type, value_multi)
        self.name = name

    def reset(self, init_value, ctype: str, py_data_type, value_multi=1, name=None):
        # 解析一下自己定的数据类型
        ctype_word_part = ctype[0]
        ctype_number_part = ctype[1:]

        # 解析解析，判断是否是sign
        if ctype_word_part.lower() == "u":
            self._signed = False
        elif ctype_word_part.lower() == "s":
            self._signed = True
        else:
            raise ValueError(f"Invalid ctype: {ctype}")
        if int(ctype_number_part) % 8 != 0:
            raise ValueError(f"Invalid ctype: {ctype}")
        if py_data_type not in [int, float, bool]:
            raise ValueError(f"Invalid var_type: {py_data_type}")

        self._byte_length = int(int(ctype_number_part) // 8)
        self._var_type = py_data_type
        self._multi = value_multi
        self._value = self._var_type(init_value)
        # self._last_update_time = time.time()
        self.name = name

        return self

    # 保护属性
    @property
    def value(self):
        return self._value

    # 修改属性（数据安全真滴牛）
    @value.setter
    def value(self, value):
        self._value = self._var_type(value)
        # self._last_update_time = time.time()

    # 放大调用
    def update_value_with_mul(self, value):
        self._value = self._var_type(value * self._multi)

    # 转换为byte输出
    @property
    def bytes(self):
        if self._multiplier != 1:
            return int(round(self._value / self._multi)).to_bytes(
                self._byte_length, "little", signed=self._signed
            )
        else:
            return int(self._value).to_bytes(
                self._byte_length, "little", signed=self._signed
            )

    # bytes的修改函数
    @bytes.setter
    def bytes(self, value):
        self._value = self._var_type(
            int.from_bytes(value, "little", signed=self._signed) * self._multiplier
        )
        self._last_update_time = time.time()

    # 长度
    @property
    def byte_length(self):
        return self._byte_length

    # 长度修改
    @byte_length.setter
    def byte_length(self, value):
        raise Exception("byte_length is read-only")

    # 时间（用不上（暂时））
    # @property
    # def last_update_time(self):
    #     return self._last_update_time
    #
    # @last_update_time.setter
    # def last_update_time(self, value):
    #     raise Exception("last_update_time is read-only")

    # 完全不知道这玩意干啥用
    @property
    def struct_fmt_type(self):
        base_dict = {1: "b", 2: "h", 4: "i", 8: "q"}
        if self._signed:
            return base_dict[self._byte_length]
        else:
            return base_dict[self._byte_length].upper()


class FC_Base_Uart_Communication:

    def __init__(self, port: str):
        super().__init__()
        self.running = False
        self.connected = False
        self._start_bit = 0xAA
        self._ser_32 = None
        self.serial_init(port)

    # 标注一下：这里的发送还是得调用SerialClass里的FC_serial
    # 稍稍魔改一下应该就能用了
    # 应答先不管
    def send_data_to_fc(self, data: bytes, option: int):
        self._set_option(option)
        sended = self._ser_32.write(data)
        return sended

    def _set_option(self, option: int) -> None:
        self._ser_32.send_config(startBit=self._start_bit, optionBit=[option])

    def serial_init(self, serial_port: str, bit_rate: int = 500000):
        # 某种意义上是初始化
        self._ser_32 = FC_Serial(serial_port, bit_rate)
        self._set_option(0)
        self.running = True

    def quit(self, joined=False) -> None:
        self.running = False
        if self._ser_32:
            self._ser_32.close()
        # logger.info("[FC] Threads closed, FC offline")
