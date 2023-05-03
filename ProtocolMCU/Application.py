import time
import threading
from others.Logger import logger
from ProtocolMCU.Protocol import class_protocol
# """
# 关于类的继承：
# 继承的类中的一切内容均直接可用
# 在继承类中写的内容一般是原类的拓展内容，基本开发顺序仍然是底层往上
# 在类中实例化的其他类永远可用，这个类往任何一个地方传都是成立的
# 并且补充一下python里的传参机制,变量一般是传地址,参量一般是传值
# 所以建立好的类就理解为是一个完整的程序，可被任何过程接入然后调用
# 不用考虑这个类传入是否会被初始化而失去其作用
# 但是不建议在两个线程中同时调用同一个类的函数,问题是,还没查，感觉上不行
# """


class class_application(class_protocol):
    def __init__(self):
        super().__init__()
        self.realtime_control_thread = None
        self.realtime_control_data_in_xyzyaw = [0, 0, 0, 0]
        self.realtime_control_running = False

    def service_start_local(self):
        """
        start service
        """
        self.start_listen_serial('/dev/ttyAMA0')

    def wait_for_connection(self, timeout_s=-1) -> bool:
        """
        等待飞控连接
        """
        t0 = time.time()
        while not self.connected:
            time.sleep(0.1)
            if 0 < timeout_s < time.time() - t0:
                logger.warning("[FC] wait for fc connection timeout")
                return False
        return True

    def wait_for_last_command_done(self, timeout_s=10) -> bool:
        """
        等待最后一次指令完成
        """
        t0 = time.time()
        time.sleep(0.5)  # 等待数据回传
        while not self.last_command_done:
            time.sleep(0.1)
            if 0 < timeout_s < time.time() - t0:
                logger.warning("[FC] wait for last command done timeout")
                return False
        return True

    def wait_for_hovering(self, timeout_s=10) -> bool:
        """
        等待进入悬停状态
        """
        t0 = time.time()
        time.sleep(0.5)  # 等待数据回传
        while not self.hovering:
            time.sleep(0.1)
            if 0 < timeout_s < time.time() - t0:
                logger.warning("[FC] wait for stabilizing timeout")
                return False
        return True

    def wait_for_lock(self, timeout_s=10) -> bool:
        """
        等待锁定
        """
        t0 = time.time()
        while self.state.unlock.value:
            time.sleep(0.1)
            if 0 < timeout_s < time.time() - t0:
                logger.warning("[FC] wait for lock timeout")
                return False
        return True

    def wait_for_takeoff_done(self, z_speed_threshold=4, timeout_s=5) -> bool:
        """
        等待起飞完成
        """
        t0 = time.time()
        time.sleep(1)  # 等待加速完成
        while self.state.vel_z.value < z_speed_threshold:
            time.sleep(0.1)
            if 0 < timeout_s < time.time() - t0:
                logger.warning("[FC] wait for takeoff done timeout")
                return False
        if self.state.alt_add.value < 20:
            logger.warning("[FC] takeoff failed, low altitude")
            return False
        time.sleep(1)  # 等待机身高度稳定
        return True

    def _realtime_control_task(self, freq):
        logger.info("[FC] realtime control task started")
        last_send_time = time.time()
        paused = False
        while self.realtime_control_running:
            while time.time() - last_send_time < 1 / freq:
                time.sleep(0.01)
            last_send_time = time.time()
            if self.state.mode.value != self.HOLD_POS_MODE:
                paused = True
                continue
            if paused:  # 取消暂停时先清空数据避免失控
                paused = False
                self.realtime_control_data_in_xyzyaw = [0, 0, 0, 0]
            try:
                self.send_realtime_control_data(
                    *self.realtime_control_data_in_xyzyaw)
            except Exception as e:
                logger.warning(f"[FC] realtime control task error: {e}")
        logger.info("[FC] realtime control task stopped")

    def start_realtime_control(self, freq: float = 15) -> None:
        """
        开始自动发送实时控制, 仅在定点模式下有效
        freq: 后台线程自动发送控制帧的频率

        警告: 除非特别需要, 否则不建议使用该方法, 而是直接调用 send_realtime_control_data,
        虽然在主线程崩溃的情况下, 子线程会因daemon自动退出, 但这一操作的延时是不可预知的

        本操作不会强行锁定控制模式于所需的定点模式, 因此可以通过切换到程控来暂停实时控制
        """
        if self.realtime_control_running:
            self.stop_realtime_control()
        self.realtime_control_running = True
        self.realtime_control_thread = threading.Thread(
            target=self._realtime_control_task, args=(freq,), daemon=True
        )
        self._thread_list.append(self.realtime_control_thread)
        self.realtime_control_thread.start()

    def stop_realtime_control(self) -> None:
        """
        停止自动发送实时控制
        """
        self.realtime_control_data_in_xyzyaw = [0, 0, 0, 0]     # 优先恢复静止
        self.realtime_control_running = False
        if self.realtime_control_thread:
            self.realtime_control_thread.join()
            self._thread_list.remove(self.realtime_control_thread)
            self.realtime_control_thread = None
        self.realtime_control_data_in_xyzyaw = [0, 0, 0, 0]

    def update_realtime_control(
            self, vel_x: int = None, vel_y: int = None,
            vel_z: int = None, yaw: int = None) -> None:
        """
        更新自动发送实时控制的目标值
        vel_x,vel_y,vel_z: cm/s 匿名坐标系
        yaw: deg/s 顺时针为正

        注意默认参数为None, 代表不更新对应的值, 若不需要建议置为0而不是留空
        """
        for n, target in enumerate([vel_x, vel_y, vel_z, yaw]):
            if target is not None:
                self.realtime_control_data_in_xyzyaw[n] = target
