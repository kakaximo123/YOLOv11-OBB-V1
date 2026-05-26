import sys, threading, socket, ctypes
import numpy as np
import cv2
from ctypes import byref, cast, POINTER, c_ubyte

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

try:
    from MvCameraControl_class import *
except Exception as e:
    print("无法导入 MvCameraControl_class，请检查 SDK")
    sys.exit(1)


# ================== 工具函数 ==================
def convert_frame(data_buf, stFrameInfo):
    """根据相机返回的像素格式转换为BGR图像"""
    h, w = stFrameInfo.nHeight, stFrameInfo.nWidth

    if stFrameInfo.enPixelType in [
        PixelType_Gvsp_BayerRG8,
        PixelType_Gvsp_BayerBG8,
        PixelType_Gvsp_BayerGB8,
        PixelType_Gvsp_BayerGR8,
    ]:
        img = np.frombuffer(data_buf, dtype=np.uint8).reshape(h, w)
        # 默认使用 RG2BGR，具体要根据你的相机 Bayer 格式改
        return cv2.cvtColor(img, cv2.COLOR_BAYER_RG2BGR)

    elif stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
        img = np.frombuffer(data_buf, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    else:
        print("⚠️ 暂不支持的像素格式:", stFrameInfo.enPixelType)
        return None


# ================== 相机线程函数 ==================
def camera_worker(ip, frame_dict, key):
    cam = MvCamera()
    device_list = MV_CC_DEVICE_INFO_LIST()
    MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)

    found = False
    for i in range(device_list.nDeviceNum):
        dev_info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        ip_str = socket.inet_ntoa(dev_info.SpecialInfo.stGigEInfo.nCurrentIp.to_bytes(4, "big"))
        if ip_str == ip:
            sel_info = dev_info
            found = True
            break

    if not found:
        print(f"未找到相机: {ip}")
        return

    # 打开相机
    cam.MV_CC_CreateHandle(sel_info)
    cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    cam.MV_CC_SetEnumValue("TriggerMode", 0)  # 连续采集
    cam.MV_CC_StartGrabbing()

    stFrameInfo = MV_FRAME_OUT_INFO_EX()
    while True:
        # 先申请一个最大缓存
        buf_size = 10 * 1024 * 1024
        data_buf = (c_ubyte * buf_size)()

        ret = cam.MV_CC_GetOneFrameTimeout(byref(data_buf), buf_size, stFrameInfo, 1000)
        if ret == 0:
            valid_bytes = stFrameInfo.nFrameLen
            frame_data = bytes(bytearray(data_buf)[:valid_bytes])
            img = convert_frame(frame_data, stFrameInfo)
            if img is not None:
                frame_dict[key] = img.copy()


# ================== PyQt 界面 ==================
class CameraApp(QWidget):
    def __init__(self, frame_dict):
        super().__init__()
        self.setWindowTitle("Industrial Camera Viewer")
        self.resize(960, 540)
        self.frame_dict = frame_dict

        self.label = QLabel("等待相机画面...", self)
        self.label.setAlignment(Qt.AlignCenter)

        layout = QHBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_frame(self):
        if "cam" in self.frame_dict:
            img = self.frame_dict["cam"]
            h, w, ch = img.shape
            qImg = QImage(img.data, w, h, ch * w, QImage.Format_BGR888)
            self.label.setPixmap(
                QPixmap.fromImage(qImg).scaled(
                    self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
            )


# ================== 主程序入口 ==================
if __name__ == "__main__":
    CAM_IP = "192.168.0.254"  # ⚠️ 换成你的工业相机 IP
    frame_dict = {}

    t = threading.Thread(target=camera_worker, args=(CAM_IP, frame_dict, "cam"), daemon=True)
    t.start()

    app = QApplication(sys.argv)
    w = CameraApp(frame_dict)
    w.show()
    sys.exit(app.exec_())
