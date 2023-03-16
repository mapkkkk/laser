# coding=UTF-8
import cv2 as cv
from time import sleep
from ProtocolMCU.Base import base_communicate
from ProtocolMCU.Application import class_application
from ProtocolMCU.Protocol import class_protocol
from PID import PID
from ImgProcess import init_cap
from ImgProcess import visualOpen
from Logger import logger
import os
import sys

'''
目前写的这一部分只是用于任务调度，可能以后就是用来执行任务用的主程序大概
注意，绝对不要让实例的名字和类的一样，不然会变得不幸
注意，在树莓派上一定要将串口改过来，两者性能差距太大了
其他组都没改是因为它们都没有使用实时控制帧做第一题
HZW
'''
# 定义串口
# 这个串口是树莓派的串口TX&RX
# 这个串口性能更高,之前分配给蓝牙,现在就分给物理串口,蓝牙就报废了
port = '/dev/ttyAMA0'


def self_reboot():
    logger.info("[MANAGER] Manager Restarting")
    os.execl(sys.executable, sys.executable, *sys.argv)


# 尝试本地连接
try:
    fc = class_application()
    fc.start_listen_serial(port)    # 建立MCU连接
    fc.wait_for_connection(5)   # 等待连接,未连接则返回错误值
except:
    logger.warning(
        "[MANAGER] Manager Connecting Failed, Reboot")
    sleep(1)
    self_reboot()

# 尝试初始化相机
try:
    cam = cv.VideoCapture(0)
    if not cam.isOpened():
        cam.open(0)
    assert cam.isOpened()
except:
    logger.warning("[MANAGER] Camera Opening Failed")
    if fc.event.key_short.is_set():
        fc.quit()
        self_reboot()

logger.info("[MANAGER] Self-Checking Passed")
###################### 开始任务 ####################
logger.info("[MANAGER] Target Mission: 1")

mission = None
# 本地模式请务必设置目标任务
target_mission = 1
try:
    if target_mission == None:
        logger.info("[MANAGER] No Target Mission Set")
    elif target_mission == 1:
        from mission1 import Mission
    # elif target_mission = 0:
        # from visionTest import vision_test

        mission = Mission(fc=fc, camera=cam)
except:
    logger.error("[MANAGER] Mission Failed, FXXK")

finally:
    if mission is not None:
        mission.stop()
    if fc.state.unlock.value:
        logger.warning("[MANAGER] Auto Landing")
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stablize()
        fc.land()
        ret = fc.wait_for_lock()
        if not ret:
            fc.lock()

################## 结束任务 ###############
fc.quit()
cam.release()
