import cv2 as cv
from ProtocolMCU.Application import class_application
from simple_pid import PID

class Mission2:
    def __init__(self, fc: class_application, camera: cv.VideoCapture):
        '''
        完成基本的实例的定义
        完成底层初始化
        完成PID的初始化(实时控制的四个维度控制PID)
        '''
        self.fc = fc
        self.cam = camera
        # self.radar = radar
        self.DEBUG = False
        self.inital_yaw = self.fc.state.yaw.value
        self.cruise_height = 120

        ########## PID ##########
        self.pid_tunings = {
            "default": (0.35, 0, 0.08),  # 导航
            "delivery": (0.4, 0.05, 0.16),  # 配送
            "landing": (0.4, 0.05, 0.16),  # 降落
        }  # PID参数 (仅导航XY使用) default settings
        self.height_pid = PID(
            0.8,
            0.0,
            0.1,
            setpoint=0,
            output_limits=(-30, 30),
            auto_mode=False,
        )
        self.height_pid.SetPoint = 140  # 定高140cm
        self.pos_x_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.pos_y_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.yaw_pid = PID(
            0.2,
            0.0,
            0.0,
            setpoint=0,
            output_limits=(-45, 45),
            auto_mode=False,
        )



        ######### FLAGS #########
        self.keep_height_flag = False  # 定高flag
        self.running_flag = False  # 运行flag
        self.thread_list = []  # 线程列表
        self.navigation_flag = False
        self.navigation_speed = 35  # 导航速度
        self.precision_speed = 25  # 精确速度
        # self.paused = False
    # vision-debug()

    def run(self):
        fc = self.fc
        cam = self.cam
        # radar = self.radar
        ########### 参数#########
        # self.running_flag = True
        # self.camera_down_pwm = 32.5
        # self.camera_up_pwm = 72
        self.navigation_speed = 35  # 导航速度
        self.precision_speed = 25  # 精确速度
        self.cruise_height = 140  # 巡航高度
        self.goods_height = 80  # 处理物品高度

        self.fc.takeoff(120)



    def moving_in_rec(self):
        for step in range(4):
            if step == 1:
                self.fc.horizontal_move(200, 25, 0)
                self.fc.wait_for_hovering()
            if step == 2:
                self.fc.horizontal_move(200, 25, 90)
                self.fc.wait_for_hovering()
            if step == 3:
                self.fc.horizontal_move(200, 25, 180)
                self.fc.wait_for_hovering()
            if step == 4;
            self.fc.horizontal_move(200, 25, 180)
            self.fc.wait_for_hovering()

    def point_landing(self):
        while(self.fc.state.vel_z != 0):
            img = self.cam.read()
            x, y = self.vision(img)
            self.fc.update_realtime_control(PID(x), PID(y))

    def circle(self):
        while(True):
            
            sleep(0.001)


    def takeoff(self, height):
        if self.

    def vision(self, img):
        pass