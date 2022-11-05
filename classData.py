# coding=UTF-8
"""
在这个文件里搞定所有常用数据结构
直接实例化即可调用
"""


class Byte_Var:
    """
    这部分定义发送数据格式，实现int向s16u16之类的转换
    到头来还是全抄了一遍，反正我自己还没到能手搓这个的境界
    """

    _value = 0
    _byte_length = 0
    _multi = 1
    _signed = False
    _var_type = None

    def __init__(self, ctype="u8", data_type=int, value_multi=1):
        self.reset(0, ctype, data_type, value_multi)

    def reset(self, init_value, ctype, py_data_type, value_multi=1):
        # 解析一下自己定的数据类型
        ctype_word_part = ctype[0]
        ctype_number_part = ctype[1:]

        # 解析解析，判断是否是sign
        if ctype_word_part.lower() == "u":
            self._signed = False
        elif ctype_word_part.lower() == "s":
            self._signed = True
        else:
            raise ValueError("Invalid ctype: {ctype}")
        if int(ctype_number_part) % 8 != 0:
            raise ValueError("Invalid ctype: {ctype}")
        if py_data_type not in [int, float, bool]:
            raise ValueError("Invalid var_type: {py_data_type}")

        self._byte_length = int(int(ctype_number_part) // 8)
        self._var_type = py_data_type
        self._multi = value_multi
        self._value = self._var_type(init_value)

        return self

    # 保护属性
    @property
    def value(self):
        return self._value

    # 修改属性
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
        if self._multi != 1:
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
            int.from_bytes(value, "little", signed=self._signed) * self._multi
        )

    # 长度
    @property
    def byte_length(self):
        return self._byte_length

    # 长度修改
    @byte_length.setter
    def byte_length(self, value):
        raise Exception("byte_length is read-only")

    # 完全不知道这玩意干啥用
    # 可能是指示数据类型用
    @property
    def struct_fmt_type(self):
        base_dict = {1: "b", 2: "h", 4: "i", 8: "q"}
        if self._signed:
            return base_dict[self._byte_length]
        else:
            return base_dict[self._byte_length].upper()


class class_option:
    take_off = None
    land = None
    lock_unlock = None
    mode_change = None
    realtime_control = None
    program_control = None
    beep = None

    def __init__(self):
        self.lock_unlock = 0x01
        self.take_off = 0x02
        self.realtime_control = 0x03
        self.land = 0x04
        self.mode_change = 0x05
        self.program_control = 0x06
        self.beep = 0x07
