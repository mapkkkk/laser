# coding=UTF-8
"""
在这个文件里搞定所有常用数据结构
直接实例化即可调用
"""
import struct


class FC_Settings_Struct:
    wait_ack_timeout = 0.1  # 应答帧超时时间
    wait_sending_timeout = 0.2  # 发送等待超时时间
    ack_max_retry = 3  # 应答失败最大重发次数
    action_log_output = True  # 是否输出动作日志
    auto_change_mode = True  # 是否自动切换飞控模式以匹配目标动作


class Byte_Var:
    """
    这部分定义发送数据格式,实现int向s16u16之类的转换
    到头来还是全抄了一遍,反正我自己还没到能手搓这个的境界
    """

    _value = 0
    _byte_length = 0
    _multi = 1.0
    _signed = False
    _var_type = None
    name = None

    def __init__(self, ctype, data_type, value_multi=1.0, name=None):
        self.reset(0, ctype, data_type, value_multi)
        self.name = name

    def reset(self, init_value, ctype, py_data_type, value_multi=1.0, name=None):
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
        self.name = name
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

    # 放大参数
    def update_value_with_mul(self, value):
        self._value = self._var_type(value * self._multi)

    # 转换为byte输出
    @property
    def bytes(self):
        if self._multi != 1:
            return int(round(self._value / self._multi)).to_bytes(  # 直接取接近的数值
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


# 选项结构体
class class_option:
    _takeoff = None
    _land = None
    _lock_unlock = None
    _mode_change = None
    _realtime_control = None
    _program_control = None
    _beep = None

    def __init__(self):
        self._lock_unlock = 0x01
        self._takeoff = 0x02
        self._realtime_control = 0x03
        self._land = 0x04
        self._mode_change = 0x05
        self._program_control = 0x06
        self._beep = 0x07

    @property
    def lock_unlock(self):
        return self._lock_unlock

    @property
    def takeoff(self):
        return self._takeoff

    @property
    def realtime_control(self):
        return self._realtime_control

    @property
    def land(self):
        return self._land

    @property
    def mode_change(self):
        return self._mode_change

    @property
    def program_control(self):
        return self._program_control

    @property
    def beep(self):
        return self._beep


class Data_To_FC:
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
    # rc_vel_x = Byte_Var("s16", int, 1)
    # rc_vel_x.value = 0
    # rc_vel_y = Byte_Var("s16", int, 1)
    # rc_vel_y.value = 0
    # rc_vel_z = Byte_Var("s16", int, 1)
    # rc_vel_z.value = 0
    # rc_yaw = Byte_Var("s16", int, 1)
    # rc_yaw.value = 0
    start_beep_data = Byte_Var("u8", int, 1)
    start_beep_data.value = 0x01
    stop_beep_data = Byte_Var("u8", int, 1)
    stop_beep_data.value = 0x02


class FC_State_Struct:
    rol = Byte_Var("s16", float, 0.01, name="rol")  # deg
    pit = Byte_Var("s16", float, 0.01, name="pit")  # deg
    yaw = Byte_Var("s16", float, 0.01, name="yaw")  # deg
    alt_fused = Byte_Var("s32", int, name="alt_fused")  # cm
    alt_add = Byte_Var("s32", int, name="alt_add")  # cm
    vel_x = Byte_Var("s16", int, name="vel_x")  # cm/s
    vel_y = Byte_Var("s16", int, name="vel_y")  # cm/s
    vel_z = Byte_Var("s16", int, name="vel_z")  # cm/s
    bat = Byte_Var("u16", float, 0.01, name="bat")  # V
    mode = Byte_Var("u8", int, name="mode")  #
    unlock = Byte_Var("u8", bool, name="unlock")  #
    cid = Byte_Var("u8", int, name="cid")  #
    cmd_0 = Byte_Var("u8", int, name="cmd_0")  #
    cmd_1 = Byte_Var("u8", int, name="cmd_1")  #

    alt = alt_add  # alias

    RECV_ORDER = [  # 数据包顺序
        rol,
        pit,
        yaw,
        alt_fused,
        alt_add,
        vel_x,
        vel_y,
        vel_z,
        bat,
        mode,
        unlock,
        cid,
        cmd_0,
        cmd_1,
    ]

    def __init__(self):
        self._fmt_string = "<" + \
            "".join([i.struct_fmt_type for i in self.RECV_ORDER])
        self._fmt_length = struct.calcsize(self._fmt_string)

    def update_from_bytes(self, bytes_data):
        if len(bytes_data) != self._fmt_length:
            raise ValueError(
                f"Invalid bytes length: {len(bytes_data)} != {self._fmt_length}"
            )
        vals = struct.unpack(self._fmt_string, bytes_data)
        for i, val in enumerate(vals):
            self.RECV_ORDER[i].update_value_with_mul(val)

    @property
    def command_now(self):
        return self.cid.value, self.cmd_0.value, self.cmd_1.value
