# coding=UTF-8
import time
from time import sleep
from classCommunicator import class_communicator
from classPID import PID
from classMission import class_mission
from imgProcess import init_cap
from imgProcess import visualOpen
'''
目前写的这一部分只是用于任务调度，可能以后就是用来执行任务用的主程序大概
注意，绝对不要让实例的名字和类的一样，不然会变得不幸
注意，在树莓派上一定要将串口改过来，两者性能差距太大了
其他组都没改是因为它们都没有使用实时控制帧做第一题
HZW
'''
# 定义串口
# 这个串口是树莓派的串口TX&RX
#
port = '/dev/ttyAMA0'

# 实例化发送类
sendclass = class_communicator(port)

# 实例化任务控制
mission = class_mission(sendclass)

# 开启摄像头
cap = init_cap()

# 初始化PID
P = 1
I = 0.2
D = 0.002
pidx = PID(P, I, D)
# 50ms更新PID
pidx.setSampleTime(0.1)

# 视觉测试代码
# while 1:
#     error_y, average_x = visualOpen(cap)
#     error_y = error_y/20
#     pidx.update(error_y)
#     control_y = pidx.output
#     print(control_y)


# PID更新周期设为10ms

state = 0
time_count = 0
current_time = 0
last_time = 0

while state is not 5:
    sleep(0.001)
    if state == 0:
        visualOpen(cap)
        # 实时控制归位
        mission.realtime_control_reset()
        sleep(0.5)
        mission.realtime_control_send()
        sleep(2)

        state = 1

    elif state == 1:
        visualOpen(cap)
        # 设置程控
        mission.mode_set(2)
        sleep(0.2)
        # 起飞
        mission.takeoff(140)
        sleep(6)
        # 前进到线的位置
        mission.program_control_move(90, 15, 0)
        sleep(12)
        mission.start_beep()
        sleep(1)
        mission.stop_beep()
        sleep(0.1)
        # 再次初始化实时控制
        mission.realtime_control_reset()
        mission.mode_set(1)
        sleep(0.2)
        mission.realtime_control_send()
        # PID控制初始化
        pidx.clear()
        current_time = time.time()
        last_time = current_time

        state = 2

    elif state == 2:
        # 更新当前时间
        current_time = time.time()
        # 持续调用视觉
        error_y, average_x = visualOpen(cap)
        # 计算控制量
        error_y = error_y / 20
        pidx.update(error_y)
        control_y = pidx.output / 5
        feedback = int(abs(control_y)/2)
        #print(control_y)
        # 更新控制量
        mission.realtime_control_config(int(control_y), 5-feedback, 0, 0)
        # 间隔20ms发送
        if current_time - last_time >= 0.05:
            mission.realtime_control_send()
            time_count = 0.05 + time_count
            last_time = current_time
        # 总共就飞6秒
        if time_count > 6:
            state = 3
            time_count = 0
            #在往回飞前停一下
            sleep(0.1)
            mission.realtime_control_reset()
            mission.realtime_control_send()
            sleep(0.4)
            mission.start_beep()
            sleep(1)
            mission.stop_beep()
            sleep(0.1)

    elif state == 3:
        # 更新当前时间
        current_time = time.time()
        # 视觉持续开启
        error_y, average_x = visualOpen(cap)
        # 计算控制量
        error_y = error_y / 20
        pidx.update(error_y)
        control_y = pidx.output / 5
        feedback = int(abs(control_y)/2)
        # 更新控制量
        mission.realtime_control_config(int(control_y), -6+feedback, 0, 0)
        # 间隔20ms发送
        if current_time - last_time >= 0.05:
            mission.realtime_control_send()
            time_count = 0.05 + time_count
            last_time = current_time
        # 就飞6秒
        if time_count > 7:
            state = 4
            mission.realtime_control_reset()
            mission.realtime_control_send()
            sleep(0.4)
            mission.mode_set(2)
            sleep(0.3)
            time_count = 0
            mission.start_beep()
            sleep(1)
            mission.stop_beep()
            sleep(0.1)

    elif state == 4:
        # 设置程控
        mission.mode_set(2)
        sleep(0.2)
        # 往回飞
        mission.program_control_move(90, 15, 180)
        sleep(9)
        # 降落
        mission.land()
        sleep(3)
        # 完事
        state = 5
