# coding=UTF-8
import cv2 as cv
from time import sleep
from ProtocolMCU.Application import class_application
from RadarDrivers_reconstruct.Radar import Radar
from RadarDrivers_reconstruct.RadarDebug import radar_debug
from others.Logger import logger
import os
import sys
# import faulthandler

import keyboard
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
DEBUG = True
# faulthandler.enable()

def self_reboot():
    logger.info("[MANAGER] Manager Restarting")
    os.execl(sys.executable, sys.executable, *sys.argv)


# 尝试本地连接
# try:
#     fc = class_application()
#     # logger.warning("[FC] init done")
#     fc.start_listen_serial(serial_port=port)  # 建立MCU连接
#     logger.warning("[FC] start listen serial")
#     fc.wait_for_connection(5)   # 等待连接,未连接则返回错误值
# except:
#     logger.warning(
#         "[MANAGER] Manager Connecting Failed, Reboot")
#     if DEBUG:
#         exit()
#     sleep(1)
#     self_reboot()

# while DEBUG:
#     sleep(5)
#     print("send_start_beep")
#     fc.unlock()
#     sleep(2)
#     fc.lock()
    # print(fc.state.pos_x.value, fc.state.pos_y.value)
    # print(fc.state.mode.value)
    # fc.set_flight_mode(fc.HOLD_POS_MODE)
    # sleep(3)
    # fc.set_flight_mode(fc.HOLD_ALT_MODE)

# 尝试初始化相机
try:
    cam = cv.VideoCapture(0)
    cam.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cam.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    fourcc = cv.VideoWriter_fourcc('M', 'J', 'P', 'G')
    cam.set(cv.CAP_PROP_FOURCC, fourcc)
    cam.set(cv.CAP_PROP_FPS, 30)
    if not cam.isOpened():
        cam.open(0)
    assert cam.isOpened()
except:
    logger.warning("[MANAGER] Camera Opening Failed")

# 尝试初始化雷达
try:
    radar = Radar()
    # radar = radar_debug()
    radar.start(
        "/dev/ttyUSB0", "LD06",
    )
except:
    logger.warning("[MANAGER] Radar Connecting Failed")

if DEBUG:
    # radar.show_radar_map()
    while DEBUG:
        pos = radar.map_visual_resolve_rt_pose(800, 1, DEBUG=True)
        logger.info(f"now pose is {pos} ")
        cv.waitKey(100)

logger.info("[MANAGER] Self-Checking Passed")
exit()
##################### 开始任务 ####################
logger.info("[MANAGER] Target Mission: 2")
# 本地模式请务必设置目标任务
mission = None
target_mission = None

try:
    if target_mission == None:
        logger.info("[MANAGER] No Target Mission Set")
    elif target_mission == 1:
        from Mission1 import Mission
    elif target_mission == 2:
        from Mission2 import Mission
    # elif target_mission = 0:
        # from visionTest import vision_test
        mission = Mission(_fc=fc, camera=cam)
        logger.info("[MANAGER] Calling Mission")

        mission.run_task_one()

        logger.info("[MANAGER] Mission Finished")
except:
    logger.error("[MANAGER] Mission Failed, FXXK")

finally:
    if mission is not None:
        mission.stop()
    if fc.state.unlock.value:
        logger.warning("[MANAGER] Auto Landing")
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stabilize()
        fc.land()
        ret = fc.wait_for_lock()
        if not ret:
            fc.lock()

################## 结束任务 ###############
fc.quit()
cam.release()
