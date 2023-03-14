import datetime
import os
import socket
import struct
import time

import cv2
import numpy
from Logger import Exception_Catcher, logger

"""
基本流程:
opencv抓取图片, 网络广播, 接收端接收, opencv显示
关于bytes到底是个啥:
字符串和 bytes 存在着千丝万缕的联系，我们可以通过字符串来创建 bytes 对象，
或者说将字符串转换成 bytes 对象。有以下三种方法可以达到这个目的：
如果字符串的内容都是 ASCII 字符,那么直接在字符串前面添加b前缀就可以转换成 bytes;
bytes 是一个类，调用它的构造方法，也就是 bytes()，可以将字符串按照指定的字符集转换成 bytes;
如果不指定字符集,那么默认采用 UTF-8。
字符串本身有一个 encode() 方法，该方法专门用来将字符串按照指定的字符集转换成对应的字节串;
如果不指定字符集，那么默认采用 UTF-8。
"""


class fps_counter:
    """
    帧率指示器
    """

    def __init__(self, max_sample=40) -> None:
        self.t = time.time()
        self.max_sample = max_sample
        self.t_list = []

    def update(self) -> None:
        self.t_list.append(time.time() - self.t)
        self.t = time.time()
        if len(self.t_list) > self.max_sample:
            self.t_list.pop(0)

    def get(self) -> float:
        length = len(self.t_list)
        sum_t = sum(self.t_list)
        if length == 0:
            return 0.0
        else:
            return length / sum_t


class net_speed_counter:
    def __init__(self, refreshTime=0.5) -> None:
        self.t = time.time()
        self.speed = 0.0
        self.sum = 0.0
        self.refreshTime = refreshTime

    def update(self, value) -> None:
        self.sum += value
        if time.time() - self.t > self.refreshTime:
            self.speed = self.sum / self.refreshTime
            self.t = time.time()
            self.sum = 0.0
            self.count = 0

    def getBps(self) -> float:
        return self.speed

    def getKbps(self) -> float:
        return self.speed / 1024

    def getKbit(self) -> float:
        return self.speed / 1024 / 8

    def getMbps(self) -> float:
        return self.speed / 1024 / 1024

    def getMbit(self) -> float:
        return self.speed / 1024 / 1024 / 8


class RT_Camera_Client:
    def __init__(
        self,
        IP_HOST,
        Port,
        Resolution=(640, 480),
        Fps=30,
        Quality=95,
        Paste_FPS=False,
        Terminate_server=False,
    ):
        self.addr = (IP_HOST, Port)
        self.resolution = Resolution
        self.fps = Fps
        self.quality = Quality
        self.timeout = 20
        self.paste_fps = Paste_FPS
        self.terminate = Terminate_server

    @Exception_Catcher
    def Socket_Connect(self):
        """
        Socket定义
        具体原理请参阅《python核心编程》

        """
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 以下完成socket连接
        self.client.connect(self.addr)
        logger.info("IP is %s:%d" % (self.addr[0], self.addr[1]))
        self.client.settimeout(self.timeout)
        self.Init_Reciver()

    def Send_Option(self):
        data = struct.pack(
            "!iiiiii",
            int(self.resolution[0]),
            int(self.resolution[1]),
            int(self.fps),
            int(self.quality),
            int(self.paste_fps),
            int(self.terminate),
        )
        logger.info(
            f"Send option: {self.resolution[0]}x{self.resolution[1]}@{self.fps}, {self.quality}% quality, Paste_FPS={self.paste_fps}, Terminate_server={self.terminate}"
        )
        self.client.send(data)

    @Exception_Catcher
    def Init_Reciver(self):
        # 按照格式打包发送帧数和分辨率
        self.Send_Option()
        if self.terminate:
            logger.info("Terminate server")
            return
        self.ip = self.addr[0]
        logger.info("%s is ready" % self.ip)
        self.packet_header = b"\x12\x23\x34\x45\x00\xff"
        self.header_len = len(self.packet_header)
        self.fps = fps_counter()
        self.netC = net_speed_counter(0.4)

    @Exception_Catcher
    def capture_frame(self):
        # 接收图像数据
        # 此处为前置的"握手"阶段
        counter = 0
        self.header_temp_buf = b""
        self.header_temp_buf += self.client.recv(self.header_len)
        while self.header_temp_buf[-self.header_len:] != self.packet_header:
            self.header_temp_buf += self.client.recv(1)
            counter += 1
            if counter > 1000000:
                raise Exception("Waiting for header timeout")
        info = struct.unpack("!i", self.client.recv(struct.calcsize("!i")))
        data_size = info[0]
        if data_size < 0 or data_size > 1000000:
            logger.error(f"Size Error: {data_size}")
            return None
        self.netC.update(data_size * 8)
        try:
            self.buf = b""  # 代表bytes类型,看不懂去看文件前面的说明
            counter = 0
            # 一直接收数据,直到接收到目标的数据量为止,目标的数据量由前置的沟通确定
            while len(self.buf) < data_size:
                self.buf += self.client.recv(data_size)  # bytes类型直接相加
                counter += 1
                if counter > 1000000:
                    raise Exception("Waiting for image timeout")
            # self.buf=self.buf[:data_size]
            # 由于先前在发送的时候用的是numpy矩阵转字符串，所以现在要转回来
            data = numpy.frombuffer(self.buf, dtype="uint8")    # 应该是将字符串转换回矩阵
            image = cv2.imdecode(data, 1)   # 矩阵按编码重新转换回图片
            self.fps.update()   # 更新fps
            return image
        except Exception as e:
            logger.error(f"Error: {e}")
            return None

    @Exception_Catcher
    def RT_Image_Recv(self):
        """
        还能咋样,有了图片就一切都好说了嘛
        """
        blk = None
        last_res = None
        while 1:
            # wait for header
            time.sleep(0.01)  # reduce CPU usage max: 100fps
            try:
                self.image = self.capture_frame()
                if self.image is None:
                    continue
                self.original_image = self.image.copy()
                width, height = self.image.shape[1], self.image.shape[0]
                if last_res is None:
                    last_res = width * height
                if last_res != width * height:
                    blk = None
                if blk is None:
                    blk = numpy.zeros(self.image.shape, numpy.uint8)
                    cv2.rectangle(
                        blk, (0, height - 50), (220,
                                                height), (255, 255, 255), -1
                    )
                    line_color = (255, 255, 255)
                    length = min(height, width) // 3
                    cv2.line(
                        blk,
                        ((width - length) // 2, height // 2),
                        ((width + length) // 2, height // 2),
                        line_color,
                        1,
                    )
                    cv2.line(
                        blk,
                        (width // 2, (height - length) // 2),
                        (width // 2, (height + length) // 2),
                        line_color,
                        1,
                    )
                    length = min(height, width)
                    cv2.rectangle(
                        blk,
                        ((width - length) // 2, (height + length) // 2),
                        ((width + length) // 2, (height - length) // 2),
                        line_color,
                        1,
                    )
                self.image = cv2.addWeighted(self.image, 1.0, blk, 0.5, 1)
                self.image = cv2.putText(
                    self.image,
                    f"{self.ip} {width}x{height}",
                    (10, height - 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (20, 20, 20),
                    1,
                )
                self.image = cv2.putText(
                    self.image,
                    f"Fps:{self.fps.get():05.2f} Net:{self.netC.getMbit():.3f}Mbit",
                    (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (20, 20, 20),
                    1,
                )
                cv2.imshow(self.ip + " Main Monitor", self.image)
            except Exception as e:
                logger.error(e)
            finally:
                key = cv2.waitKey(1)
                if key == 27:  # 按‘ESC’（27）退出
                    self.client.close()
                    cv2.destroyAllWindows()
                    break
                elif key == ord("s"):
                    time_string = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    if not os.path.exists("cv_saved_images"):
                        os.mkdir("cv_saved_images")
                    cv2.imwrite(
                        f"cv_saved_images/{time_string}.jpg", self.original_image
                    )
                    logger.info(f"Saved image {time_string}.jpg")
                    time.sleep(0.1)


"""
帧率组合:
640x360@30
800x600@20
1024x768@10
1280x720@10
1920x1080@5
"""

if __name__ == "__main__":
    camera = RT_Camera_Client(
        IP_HOST="192.168.137.27",
        # IP="raspberrypi",
        Port=6756,
        Resolution=(800, 600),
        Fps=60,
        Paste_FPS=0,
        Quality=70,
        Terminate_server=0,
    )
    logger.info("Waiting for server...")
    while True:
        try:
            camera.Socket_Connect()
        except:
            continue
        break
    camera.RT_Image_Recv()
