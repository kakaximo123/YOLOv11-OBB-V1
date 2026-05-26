# 导入所需的系统和库模块
import sys, threading, socket, ctypes, os, time  # 系统操作、线程、网络、ctypes等基础模块
import numpy as np  # 用于数组处理，常用于图像数据处理
import cv2  # OpenCV库，用于图像处理
from ctypes import byref, cast, POINTER, c_ubyte  # ctypes相关函数，用于处理C类型数据

# PyQt5相关模块，用于创建图形界面
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

# 尝试导入相机控制类，该类通常来自相机厂商提供的SDK
try:
    from MvCameraControl_class import *  # 相机控制相关的类和函数
except Exception as e:
    print("无法导入 MvCameraControl_class，请检查 SDK")  # 导入失败时提示
    sys.exit(1)  # 退出程序

# ================== 全局信号 ==================
capture_signal = 0  # 全局变量，用于控制截图：1表示需要截图，0表示不需要

# ================== 工具函数 ==================
def convert_frame(data_buf, stFrameInfo):
    """
    将相机获取的原始数据转换为OpenCV可处理的图像格式
    data_buf: 原始图像数据缓冲区
    stFrameInfo: 包含图像信息的结构体（如宽、高、像素格式等）
    """
    h, w = stFrameInfo.nHeight, stFrameInfo.nWidth  # 获取图像的高度和宽度

    # 判断相机输出的像素格式，进行相应的转换
    if stFrameInfo.enPixelType in [
        PixelType_Gvsp_BayerRG8,
        PixelType_Gvsp_BayerBG8,
        PixelType_Gvsp_BayerGB8,
        PixelType_Gvsp_BayerGR8,
    ]:
        # 如果是拜耳格式，转换为BGR格式（OpenCV默认的彩色格式）
        img = np.frombuffer(data_buf, dtype=np.uint8).reshape(h, w)  # 将缓冲区数据转换为numpy数组
        return cv2.cvtColor(img, cv2.COLOR_BAYER_RG2BGR)  # 拜耳格式转BGR

    elif stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
        # 如果是单通道灰度格式，转换为BGR格式（为了统一显示格式）
        img = np.frombuffer(data_buf, dtype=np.uint8).reshape(h, w)  # 转换为numpy数组
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)  # 灰度转BGR

    else:
        # 不支持的像素格式
        print("⚠️ 暂不支持的像素格式:", stFrameInfo.enPixelType)
        return None

def bayer_to_bgr(raw, h, w, pixel_type):
    mapping = {
        PixelType_Gvsp_BayerRG8: cv2.COLOR_BAYER_RG2BGR,
        PixelType_Gvsp_BayerBG8: cv2.COLOR_BAYER_BG2BGR,
        PixelType_Gvsp_BayerGB8: cv2.COLOR_BAYER_GB2BGR,
        PixelType_Gvsp_BayerGR8: cv2.COLOR_BAYER_GR2BGR,
    }
    code = mapping.get(pixel_type)
    if code is None:
        return None
    img_mono = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
    return cv2.cvtColor(img_mono, code)


def mono8_to_bgr(raw, h, w):
    return cv2.cvtColor(np.frombuffer(raw, dtype=np.uint8).reshape(h, w), cv2.COLOR_GRAY2BGR)

# ================== 相机线程 ==================
def camera_worker(ip, frame_dict, key):
    """
    相机工作线程函数，负责连接相机、获取图像并处理
    ip: 相机的IP地址
    frame_dict: 用于存储图像数据的字典（线程间共享）
    key: 存储在字典中的键名
    """
    global capture_signal  # 声明使用全局变量capture_signal

    cam = MvCamera()  # 创建相机对象
    device_list = MV_CC_DEVICE_INFO_LIST()  # 设备信息列表结构体

    # 枚举所有可用设备（千兆网设备和USB设备）
    MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)

    found = False  # 标记是否找到目标相机
    # 遍历设备列表，寻找指定IP的相机
    for i in range(device_list.nDeviceNum):
        # 获取第i个设备的信息
        dev_info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        # 将相机IP从整数转换为字符串格式
        ip_bytes = dev_info.SpecialInfo.stGigEInfo.nCurrentIp.to_bytes(4, "big")
        ip_str = socket.inet_ntoa(ip_bytes)
        # 找到匹配IP的相机
        if ip_str == ip:
            sel_info = dev_info  # 保存找到的设备信息
            found = True
            break

    if not found:
        print(f"未找到相机: {ip}")  # 未找到相机时提示
        return  # 退出线程

    # 打开相机的一系列操作
    cam.MV_CC_CreateHandle(sel_info)  # 创建相机句柄
    cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)  # 独占模式打开相机
    cam.MV_CC_SetEnumValue("TriggerMode", 0)  # 设置触发模式为0（连续采集模式）
    cam.MV_CC_StartGrabbing()  # 开始采集图像

    stFrameInfo = MV_FRAME_OUT_INFO_EX()  # 用于存储帧信息的结构体

#---------------------------------------------------------------------------------------------------------------------------------------
    stParam = MVCC_INTVALUE()
    cam.MV_CC_GetIntValue("PayloadSize", stParam)
    payload_size = stParam.nCurValue
    data_buf = (ctypes.c_ubyte * payload_size)()
    frame_info = MV_FRAME_OUT_INFO_EX()

    ret = cam.MV_CC_GetOneFrameTimeout(data_buf, payload_size, frame_info, 1000)
    if ret == 0:
        w, h = frame_info.nWidth, frame_info.nHeight
        raw = (ctypes.c_ubyte * frame_info.nFrameLen).from_buffer(data_buf)
        if frame_info.enPixelType == PixelType_Gvsp_Mono8:
            img = mono8_to_bgr(raw, h, w)
        else:
            img = bayer_to_bgr(raw, h, w, frame_info.enPixelType)
        if img is None:
            img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    h, w = img.shape[:2]
    print(h)

    # ==== ROI 定义 ====
    roi_x_left = 20
    roi_x_right = 1660
    roi_w_px = roi_x_right - roi_x_left
    roi = img[:, roi_x_left:roi_x_right].copy()

    # ✅✅ 相机1尺度 ✅✅
    real_width_cm = 60.09
    real_height_cm = 74.00
    pixels_per_cm_x = roi_w_px / real_width_cm
    pixels_per_cm_y = h / real_height_cm



#-----------------------------------------------------------------------------------------------------------------------------

    # 创建截图保存目录
    save_dir = "./photo"  # 截图保存路径
    os.makedirs(save_dir, exist_ok=True)  # 若目录不存在则创建，exist_ok=True避免已存在时报错

    # 循环获取图像
    while True:
        buf_size = 10 * 1024 * 1024  # 缓冲区大小（10MB）
        data_buf = (c_ubyte * buf_size)()  # 创建缓冲区
        # 从相机获取一帧图像，超时时间1000ms
        ret = cam.MV_CC_GetOneFrameTimeout(byref(data_buf), buf_size, stFrameInfo, 1000)
        if ret == 0:  # 如果成功获取图像（返回值0通常表示成功）
            valid_bytes = stFrameInfo.nFrameLen  # 获取有效数据长度
            # 将缓冲区数据转换为bytes类型（只取有效长度部分）
            frame_data = bytes(bytearray(data_buf)[:valid_bytes])
            # 转换图像格式
            img = convert_frame(frame_data, stFrameInfo)
            cv2.line(img, (roi_x_right, 0), (0, 0), (0, 0, 0), 3)
            for cm in range(0, int(real_width_cm) + 1, 5):
                x_pos = int(roi_x_right - cm * pixels_per_cm_x)
                cv2.line(img, (x_pos, 0), (x_pos, 25), (0, 0, 0), 2)
                cv2.putText(img, f"{cm}", (x_pos - 25, 45),
                            cv2.FONT_HERSHEY_TRIPLEX, 1, (50, 50, 50), 2)

            cv2.line(img, (roi_x_right, 0), (roi_x_right, h), (0, 0, 0), 3)
            for cm in range(0, int(real_height_cm) + 1, 5):
                y_pos = int(cm * pixels_per_cm_y)
                cv2.line(img, (roi_x_right - 25, y_pos), (roi_x_right, y_pos), (0, 0, 0), 2)
                cv2.putText(img, f"{-cm}", (roi_x_right - 90, y_pos + 10),
                            cv2.FONT_HERSHEY_TRIPLEX, 1, (50, 50, 50), 2)
            if img is not None:  # 如果转换成功
                frame_dict[key] = img.copy()  # 将图像存入共享字典

                # 检查是否需要截图（当capture_signal为1时）
                if capture_signal == 1:
                    # 生成带时间戳的文件名，避免重名
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(save_dir, f"capture_{timestamp}.jpg")
                    # OpenCV保存图像默认是BGR格式，但有些情况下可能需要转换
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    cv2.imwrite(filename, img_rgb)  # 保存图像
                    print(f"✅ 已保存截图: {filename}")
                    capture_signal = 0  # 截图完成后重置信号




# ================== 终端输入线程 ==================
def input_listener():
    """监听终端输入，当用户按回车键时触发截图信号"""
    global capture_signal  # 声明使用全局变量
    print("💡 按回车键截图...")  # 提示用户操作方式
    while True:
        input()  # 等待用户输入（按回车键）
        capture_signal = 1  # 设置截图信号
        print("📸 截图信号置位")  # 提示信号已设置

# ================== PyQt 界面 ==================
class CameraApp(QWidget):
    """相机显示界面类，继承自QWidget"""
    def __init__(self, frame_dict):
        super().__init__()  # 调用父类构造函数
        self.setWindowTitle("Industrial Camera Viewer")  # 设置窗口标题
        self.resize(960, 540)  # 设置窗口初始大小
        self.frame_dict = frame_dict  # 保存共享的图像字典引用

        # 创建用于显示图像的标签
        self.label = QLabel("等待相机画面...", self)  # 初始显示文本
        self.label.setAlignment(Qt.AlignCenter)  # 文本居中显示

        # 设置布局
        layout = QHBoxLayout()  # 水平布局
        layout.addWidget(self.label)  # 将标签添加到布局
        self.setLayout(layout)  # 设置窗口布局

        # 创建定时器，用于定期更新显示
        self.timer = QTimer()
        # 定时器超时信号连接到update_frame方法（每30ms更新一次）
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 启动定时器，间隔30ms

    def update_frame(self):
        """更新显示的图像"""
        # 检查字典中是否有相机图像数据
        if "cam" in self.frame_dict:
            img = self.frame_dict["cam"]  # 获取图像数据
            h, w, ch = img.shape  # 获取图像的高度、宽度和通道数
            # 将OpenCV图像（BGR格式）转换为QImage格式，用于PyQt显示
            qImg = QImage(img.data, w, h, ch * w, QImage.Format_BGR888)
            # 将QImage转换为QPixmap，并设置到标签上，保持比例缩放
            self.label.setPixmap(
                QPixmap.fromImage(qImg).scaled(
                    self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
            )

# ================== 主程序入口 ==================
if __name__ == "__main__":
    CAM_IP = "169.254.0.102"  # 相机的IP地址，需要根据实际情况修改
    frame_dict = {}  # 用于在线程间共享图像数据的字典

    # 创建并启动相机线程
    # daemon=True表示该线程是守护线程，主程序退出时自动结束
    t_cam = threading.Thread(target=camera_worker, args=(CAM_IP, frame_dict, "cam"), daemon=True)
    t_cam.start()  # 启动线程

    # 创建并启动终端输入监听线程
    t_input = threading.Thread(target=input_listener, daemon=True)
    t_input.start()  # 启动线程

    # 启动PyQt应用程序
    app = QApplication(sys.argv)  # 创建应用实例
    w = CameraApp(frame_dict)  # 创建主窗口
    w.show()  # 显示窗口
    sys.exit(app.exec_())  # 进入应用主循环，等待用户操作