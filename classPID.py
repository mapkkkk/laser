# coding=UTF-8
import time

'''
PID控制
这个类写的好，直接拿来爽用
'''


class PID:

    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None):

        self.output = None
        self.windup_guard = None
        self.int_error = None
        self.SetPoint = None
        self.last_error = None
        self.DTerm = None
        self.ITerm = None
        self.PTerm = None
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        # 理解为归零
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        # 偏置
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.current_time = time.time()
        self.last_time = self.current_time

        # Windup Guard
        # 积分上限还是给小一些，不然容易过冲
        self.int_error = 0.0
        self.windup_guard = 10.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i /int_{0}^{t} e(t)dt + k_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        # 这里定义的error是指的是误差，不是啥错误
        # 然后这个feedback_value就是新的误差值
        error = self.SetPoint - feedback_value
        # 时间上的更新
        self.current_time = current_time if current_time is not None else time.time()
        # 时间的间隔量
        delta_time = self.current_time - self.last_time
        # 微分将要用到的误差的变化量
        delta_error = error - self.last_error
        # 如果时间差到了更新的时候，进到这个里面去更新PID输出，不然就按照原先的量输出
        if delta_time >= self.sample_time:
            # 误差量
            self.PTerm = self.Kp * error
            # 积分量
            self.ITerm += error * delta_time
            # 设置积分量上限
            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard
            # 微分量
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error
            # 输出，认为直接访问这个类里的输出量就可以丢给飞控用了
            # 认为需要三个PID：一个是y轴速度（水平平移），一个是x轴速度（控制与线的距离），最后一个是旋转量raw
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in set_point occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sample time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time
