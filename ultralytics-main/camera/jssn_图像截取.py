import sys, threading, socket, ctypes, os, time
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

# ================== 全局信号 ==================
capture_signal = 0  # 外部置位 1 表示截图

# ================== 工具函数 ==================
def convert_frame(data_buf, stFrameInfo):
    h, w = stFrameInfo.nHeight, stFrameInfo.nWidth

    if stFrameInfo.enPixelType in [
        PixelType_Gvsp_BayerRG8,
        PixelType_Gvsp_BayerBG8,
        PixelType_Gvsp_BayerGB8,
        PixelType_Gvsp_BayerGR8,
    ]:
        img = np.frombuffer(data_buf, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_BAYER_RG2BGR)

    elif stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
        img = np.frombuffer(data_buf, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    else:
        print("⚠️ 暂不支持的像素格式:", stFrameInfo.enPixelType)
        return None

# ================== 相机线程 ==================
def camera_worker(ip, frame_dict, key):
    global capture_signal

    cam = MvCamera()
    device_list = MV_CC_DEVICE_INFO_LIST()
    MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)

    found = False
    for i in range(device_list.nDeviceNum):
        dev_info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        ip_bytes = dev_info.SpecialInfo.stGigEInfo.nCurrentIp.to_bytes(4, "big")
        ip_str = socket.inet_ntoa(ip_bytes)
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

    # 创建截图目录
    save_dir = "./photo"
    os.makedirs(save_dir, exist_ok=True)

    while True:
        buf_size = 10 * 1024 * 1024
        data_buf = (c_ubyte * buf_size)()
        ret = cam.MV_CC_GetOneFrameTimeout(byref(data_buf), buf_size, stFrameInfo, 1000)
        if ret == 0:
            valid_bytes = stFrameInfo.nFrameLen
            frame_data = bytes(bytearray(data_buf)[:valid_bytes])
            img = convert_frame(frame_data, stFrameInfo)
            if img is not None:
                frame_dict[key] = img.copy()

                # 如果截图信号为 1，则保存图片
                if capture_signal == 1:
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(save_dir, f"capture_{timestamp}.jpg")
                    cv2.imwrite(filename, img)
                    print(f"✅ 已保存截图: {filename}")
                    capture_signal = 0  # 保存完成自动清零

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
    CAM_IP = "169.254.0.102"  # ⚠️ 替换成你的相机 IP
    frame_dict = {}

    t = threading.Thread(target=camera_worker, args=(CAM_IP, frame_dict, "cam"), daemon=True)
    t.start()

    app = QApplication(sys.argv)
    w = CameraApp(frame_dict)
    w.show()

    # 模拟外部置位截图信号
    def trigger_capture():
        global capture_signal
        capture_signal = 1
        print("📸 截图信号置位")

    # 每 5 秒触发一次截图（示例）
    capture_timer = QTimer()
    capture_timer.timeout.connect(trigger_capture)
    capture_timer.start(5000)

    sys.exit(app.exec_())
