# -- coding: utf-8 --
import threading
import ctypes
import time
import os
import struct
from ctypes import *
from datetime import datetime
import cv2
import numpy as np
import math
import torch

# 导入 SDK 模块
from Mv3dRgbdImport.Mv3dRgbdDefine import *
from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import (
    DeviceType_Ethernet, DeviceType_USB,
    DeviceType_Ethernet_Vir, DeviceType_USB_Vir,
    MV3D_RGBD_OK, ImageType_Depth, ImageType_RGB8_Planar
)

from ultralytics import YOLO
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext

Model = YOLO(r'E:\cumt\jianza\ultralytics-main\ultralytics-main\runs\obb\train24\weights\best.pt')
npy_path = r"E:\cumt\jianza\depth\raw\depth_20260402.npy"
depth_base = np.load(npy_path)


class ModbusWriter:
    """
    电脑端作为 Modbus TCP 从站，只发送一次数据供工控机读取
    """

    REG_X = 0
    REG_Y = 2
    REG_Z = 4
    REG_SPEED = 6
    REG_RY = 8
    REG_RZ = 10

    REG_9_FLAG = 12
    REG_10_FLAG = 14
    REG_11_FLAG = 16
    REG_12_FLAG = 18

    def __init__(self, ip="0.0.0.0", port=502):
        self.ip = ip
        self.port = port
        self.context = self._init_context()
        print(f"✅ Modbus 从站初始化完成，监听地址：{ip}:{port}")

    @staticmethod
    def float_to_regs(value, order="BADC"):
        """浮点数 -> 2个寄存器"""
        b = struct.pack('>f', float(value))
        if order == "BADC":
            b = b[1:2] + b[0:1] + b[3:4] + b[2:3]
        elif order == "CDAB":
            b = b[2:4] + b[0:2]
        return list(struct.unpack('>HH', b))

    @staticmethod
    def _init_context():
        """初始化 Modbus 数据存储区"""
        store = ModbusSlaveContext(
            hr=ModbusSequentialDataBlock(0, [0] * 120)
        )
        return ModbusServerContext(slaves=store, single=True)

    def send_position_6axis(self, x, y, z, rs, ry, rz, float_order="BADC"):
        """一次性写入机械臂六轴位置寄存器（供主站读取）"""
        slave_id = 0x01
        regs_start = [self.REG_X, self.REG_Y, self.REG_Z,
                      self.REG_SPEED, self.REG_RY, self.REG_RZ]
        values = [x, y, z, rs, ry, rz]
        regs = []
        for v in values:
            regs.extend(self.float_to_regs(v, order=float_order))
        self.context[slave_id].setValues(3, regs_start[0], regs)
        print(f"✅ 已写入机械臂6轴数据：{[round(v, 3) for v in values]}")

    def send_all_flag(self, flag9, flag10, flag11, flag12, float_order="BADC"):
        """一次性写入标志位寄存器"""
        slave_id = 0x01
        regs_start = [self.REG_9_FLAG, self.REG_10_FLAG,
                      self.REG_11_FLAG, self.REG_12_FLAG]
        values = [flag9, flag10, flag11, flag12]
        regs = []
        for v in values:
            regs.extend(self.float_to_regs(v, order=float_order))
        self.context[slave_id].setValues(3, regs_start[0], regs)
        print(f"✅ 已写入标志位数据：{[round(v, 3) for v in values]}")

    def start_server(self):
        """启动 Modbus 从站服务器（后台线程方式）"""

        def run_server():
            print("🚀 启动 Modbus TCP 从站服务器，等待工控机连接...")
            StartTcpServer(self.context, address=(self.ip, self.port))

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
        print("✅ 从站服务器已在后台启动，可继续执行其他任务")


class ImageHandler:
    roi_x_left, roi_x_right = 200, 800
    roi_y_left, roi_y_right = 350, 740
    pixels_per_mm = 1060 / 600
    pixels_per_cm = 106 / 600

    def __init__(self, model, modbus=None, conf_thresh=0.75, device="cuda"):
        self.lock = threading.Lock()

        self.model = model
        self.device = device
        self.conf_thresh = conf_thresh
        self.modbus = modbus

        self.dot_list = []
        self.targets = None
        self.object_line = {"line0": False, "line1": False, "line2": False}
        self.line1_time = 0.0
        self.object_count = 0
        self.LAST_FLAG1 = True
        self.SPEED_NOW = 0.0
        self.delay_time = 0.0
        self.cz_mm = 20.0
        self.dot_list_maxlen = 30
        self.image = None
        self.h, self.w = None, None
        self.roi = None
        self.speed_fit_min_points = 5
        self.speed_fit_window = 12
        self.speed_min_displacement_px = 8
        self.dep = None

        # 新增：用于区分“新物体刚进入”与“同一物体持续跟踪”
        self.object_active = False

    def draw_lines(self, image):
        self.image = image
        self.roi = self.image[350:740, 200:800].copy()
        self.h, self.w = self.roi.shape[:2]

        with self.lock:
            img = self.image
            cv2.line(img, (200, 350), (800, 350), (0, 0, 255), 2)
            cv2.line(img, (200, 740), (800, 740), (0, 0, 255), 2)
            for cm in range(0, 105, 5):
                x_pos = int(200 + cm / ImageHandler.pixels_per_cm)
                cv2.line(img, (x_pos, 350), (x_pos, 370), (0, 0, 255), 2)
                cv2.putText(img, f"{cm}", (x_pos - 10, 345), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (100, 255, 0), 1)

            cv2.line(img, (200, 740), (200, 350), (0, 0, 255), 2)
            cv2.line(img, (800, 740), (800, 350), (0, 0, 255), 2)
            for cm in range(0, 68, 5):
                y_pos = int(350 + cm / ImageHandler.pixels_per_cm)
                cv2.line(img, (200, y_pos), (220, y_pos), (0, 0, 255), 2)
                cv2.putText(img, f"{cm}", (175, y_pos), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (100, 255, 0), 1)

            cv2.line(img, (300, 740), (300, 350), (255, 0, 0), 2)
            cv2.putText(img, "line_IN", (250, 330), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0), 1)

            cv2.line(img, (700, 740), (700, 350), (255, 0, 0), 2)
            cv2.putText(img, "line_OUT", (650, 330), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0), 1)

            return img

    def _compute_speed_mm_per_s(self):
        """
        使用最近轨迹点做线性拟合，得到更稳定的速度（mm/s）
        """
        if len(self.dot_list) < self.speed_fit_min_points:
            return 0.0

        points = self.dot_list[-self.speed_fit_window:]

        # 这里实际记录的是 cx，不是 cy
        cx_vals = np.array([p[0] for p in points], dtype=np.float64)
        t_vals = np.array([p[1] for p in points], dtype=np.float64)

        unique_mask = np.concatenate(([True], np.diff(t_vals) > 1e-6))
        cx_vals = cx_vals[unique_mask]
        t_vals = t_vals[unique_mask]

        if len(cx_vals) < self.speed_fit_min_points:
            return 0.0

        if (t_vals[-1] - t_vals[0]) < 1e-3:
            return 0.0

        if abs(cx_vals[-1] - cx_vals[0]) < self.speed_min_displacement_px:
            return 0.0

        try:
            v_px_per_s, _ = np.polyfit(t_vals, cx_vals, 1)
        except Exception as e:
            print("[WARN] speed polyfit failed:", e)
            return 0.0

        speed_mm_per_s = abs(v_px_per_s) * ImageHandler.pixels_per_mm

        if not np.isfinite(speed_mm_per_s):
            return 0.0

        return float(speed_mm_per_s)

    def detection_logic(self, depth_img=None):
        with self.lock:
            now = time.time()
            try:
                with torch.no_grad():
                    results = self.model(self.roi, device=self.device, conf=self.conf_thresh, verbose=False)
            except Exception as e:
                print("[ERROR] 推理失败：", e)
                self.image = None
                return self.image

        r = results[0]
        if r.obb is not None and len(r.obb.xywhr) > 0:
            boxes = r.obb.xywhr.cpu().numpy()
            classes = r.obb.cls.cpu().numpy().astype(int)
            confs = r.obb.conf.cpu().numpy()

            for box, cls_id, conf in zip(boxes, classes, confs):
                name = self.model.names[cls_id]
                cx, cy, w, h, angle = box

                abs_cx = int(cx + 200)
                abs_cy = int(cy + 350)

                angle_deg = math.degrees(angle)
                if h > w:
                    angle_deg += 90.0
                angle_deg = (angle_deg + 180) % 360 - 180
                if angle_deg > 90:
                    angle_deg -= 180
                elif angle_deg < -90:
                    angle_deg += 180
                angle_deg = -angle_deg

                print(f"{name}: cx={abs_cx:.1f}, cy={abs_cy:.1f}, w={w:.1f}, h={h:.1f}, 长边角度={angle_deg:.1f}°,置信度={conf:.2f}")

                cv2.circle(self.image, (abs_cx, abs_cy), 6, (0, 0, 255), 2)
                cv2.putText(self.image, f"{name, abs_cx, abs_cy}", (abs_cx + 5, abs_cy - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 1)

                rect = ((abs_cx, abs_cy), (int(w), int(h)), math.degrees(angle))
                box_pts = cv2.boxPoints(rect)
                box_pts = np.intp(box_pts)
                cv2.drawContours(self.image, [box_pts], 0, (0, 0, 255), 2)

                cx_mm = cx * ImageHandler.pixels_per_mm
                cy_mm = cy * ImageHandler.pixels_per_mm

                # 只有当前活动目标的轨迹会被记录
                self.dot_list.append((cx, now))
                if len(self.dot_list) > self.dot_list_maxlen:
                    self.dot_list.pop(0)

                if self.targets is None:
                    self.targets = {
                        "last_line1_time": None,
                        "last_line2_time": None,
                        "last_line1_flag": None,
                        "last_line2_flag": None,
                        "frame_count": 0,
                    }

                self.targets["frame_count"] += 1

                if depth_img is not None:
                    # 修复：depth_base 第二个索引应为 abs_cx
                    depth_length = -abs(int(depth_base[abs_cy, abs_cx]) - int(depth_img[abs_cy, abs_cx]))
                    self.dep = depth_length
                    print("真实高度")
                    self.cz_mm = self.dep * 3 / 5
                    if abs(self.cz_mm) < 30:
                        self.cz_mm = -30

                label = f"({cx_mm:.1f}mm, {cy_mm:.1f}mm, {self.dep:.1f}mm)"
                cv2.putText(self.image, label, (abs_cx + 5, abs_cy + 40),
                            cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1)

                if cx > 100:
                    # 新物体第一次进入时，清空上一轮测速轨迹，避免第二遍测速被污染
                    if not self.object_active:
                        self.object_active = True
                        self.dot_list.clear()
                        self.targets = {
                            "last_line1_time": None,
                            "last_line2_time": None,
                            "last_line1_flag": None,
                            "last_line2_flag": None,
                            "frame_count": 0,
                        }
                        self.object_line = {"line0": False, "line1": False, "line2": False}
                        self.line1_time = 0.0
                        self.delay_time = 0.0
                        self.SPEED_NOW = 0.0

                        # 当前帧作为新轨迹第一个点重新写入
                        self.dot_list.append((cx, now))

                    self.object_line["line0"] = True
                    if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                        try:
                            self.modbus.send_all_flag(0, 0, 0, 0)
                        except Exception as e:
                            print("[WARN] send_all_flag failed:", e)
                    print("检测到物体")

                if 100 <= cx <= 500 and self.object_line["line0"]:
                    self.object_line["line1"] = True
                    self.line1_time = time.time()
                    if self.targets["last_line1_flag"] is None:
                        self.targets["last_line1_time"] = time.time()
                        self.targets["last_line1_flag"] = True

                    if self.modbus is not None and hasattr(self.modbus, "send_position_6axis"):
                        try:
                            print("高度", self.dep)
                            # self.modbus.send_position_6axis(cx_mm, cy_mm, self.cz_mm, 0, 0, -angle_deg + 90)
                            self.modbus.send_position_6axis(cx_mm, cy_mm, -140, 0, 0, -angle_deg + 90)
                        except Exception as e:
                            print("[WARN] send_position_6axis failed:", e)

                    if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                        try:
                            self.modbus.send_all_flag(1, 0, 0, 0)
                            print("随动阶段")
                        except Exception as e:
                            print("[WARN] send_all_flag failed:", e)

                if cx > 500 and self.object_line["line1"]:
                    self.object_line["line2"] = True
                    if self.targets["last_line2_flag"] is None:
                        self.targets["last_line2_flag"] = True
                        self.SPEED_NOW = self._compute_speed_mm_per_s()
                        print("速度mm/s:", round(self.SPEED_NOW, 2))

                        if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                            try:
                                for i in range(5):
                                    self.modbus.send_all_flag(1, 1, 0, 1)
                                    print("跳出随动")
                            except Exception as e:
                                print("[WARN] send_all_flag failed:", e)

                        self.targets["last_line2_time"] = time.time()

                        try:
                            self.delay_time = 340 / self.SPEED_NOW if self.SPEED_NOW != 0 else 0
                        except Exception as e:
                            self.delay_time = 0
                            print(e)
                        print("速度mm/s, 触发等待时间", round(self.SPEED_NOW, 2), self.delay_time)
                        if self.SPEED_NOW > 500:
                            self.delay_time = self.delay_time + 0.1

        if self.object_line["line2"]:
            if self.targets and self.targets.get("last_line2_time") is not None:
                print("123")
                print(self.targets["last_line2_time"])

                if time.time() - self.targets["last_line2_time"] > self.delay_time:
                    for i in range(1, 5):
                        print(f"开始跟随,物体{self.object_count}")
                    if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                        try:
                            for i in range(5):
                                self.modbus.send_all_flag(1, 1, 1, 0, float_order="BADC")
                                print("抓取指令")
                            time.sleep(0.1)
                        except Exception as e:
                            print("[WARN] send_all_flag failed:", e)

                    for key in self.object_line:
                        self.object_line[key] = False
                    self.targets = None
                    self.LAST_FLAG1 = True
                    self.cz_mm = 20
                    self.delay_time = 0
                    self.SPEED_NOW = 0

                    # 关键修复：正常完成一轮后也必须清空轨迹
                    self.dot_list.clear()
                    self.object_active = False

        if self.object_line["line1"] and not self.object_line["line2"]:
            if time.time() - self.line1_time > 10:
                for key in self.object_line:
                    self.object_line[key] = False
                self.targets = None
                self.LAST_FLAG1 = True
                self.cz_mm = 20
                self.delay_time = 0
                self.SPEED_NOW = 0
                self.dot_list.clear()
                self.object_active = False
                if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                    try:
                        self.modbus.send_all_flag(0, 0, 0, 0, float_order="BADC")
                        self.modbus.send_position_6axis(0, 0, 0, 0, 0, 0, float_order="BADC")
                    except Exception as e:
                        print("[WARN] send_all_flag/send_position_6axis failed:", e)
                for i in range(10):
                    print("出现异常，进行清零操作")

        return self.image


class CameraHandler:
    def __init__(self, device_index=0):
        self.camera = None
        self.device_index = device_index
        self.stop_flag = False

        self.save_dir = r"E:\cumt\jianza\depth"
        self.depth_raw_dir = os.path.join(self.save_dir, "raw")
        self.depth_vis_dir = os.path.join(self.save_dir, "vis")
        os.makedirs(self.depth_raw_dir, exist_ok=True)
        os.makedirs(self.depth_vis_dir, exist_ok=True)

        self.Handler = ImageHandler(model=Model, modbus=Modbus)

    def save_depth_frame(self, depth_img):
        """单独保存原始3D深度数据"""
        if depth_img is None:
            print("[WARN] 当前没有深度图可保存")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        raw_path_npy = os.path.join(self.depth_raw_dir, f"depth_{timestamp}.npy")
        np.save(raw_path_npy, depth_img)

        raw_path_png = os.path.join(self.depth_raw_dir, f"depth_{timestamp}.png")
        cv2.imwrite(raw_path_png, depth_img)

        depth_norm = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
        depth_colormap = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
        vis_path = os.path.join(self.depth_vis_dir, f"depth_vis_{timestamp}.png")
        cv2.imwrite(vis_path, depth_colormap)

        print(f"[INFO] 深度原始数据已保存: {raw_path_npy}")
        print(f"[INFO] 深度16位PNG已保存: {raw_path_png}")
        print(f"[INFO] 深度可视化图已保存: {vis_path}")

    def open_camera(self):
        """打开相机"""
        nDeviceNum = ctypes.c_uint(0)
        ret = Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(
            DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir,
            byref(nDeviceNum)
        )
        if ret != 0 or nDeviceNum.value == 0:
            raise RuntimeError("没有检测到相机设备")

        stDeviceList = MV3D_RGBD_DEVICE_INFO_LIST()
        Mv3dRgbd.MV3D_RGBD_GetDeviceList(
            DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir,
            pointer(stDeviceList.DeviceInfo[0]), 20, byref(nDeviceNum)
        )

        self.camera = Mv3dRgbd()
        ret = self.camera.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[self.device_index]))
        if ret != 0:
            raise RuntimeError("打开相机失败")

        ret = self.camera.MV3D_RGBD_Start()
        if ret != 0:
            self.camera.MV3D_RGBD_CloseDevice()
            raise RuntimeError("启动取流失败")

        print("[INFO] 相机已启动")

    def stream_frames(self):
        """循环获取并显示彩色图和深度图"""
        if self.camera is None:
            raise RuntimeError("相机未打开")

        print("[INFO] 开始实时取流，按 's' 保存深度图，按 'q' 退出")

        while not self.stop_flag:
            stFrameData = MV3D_RGBD_FRAME_DATA()
            ret = self.camera.MV3D_RGBD_FetchFrame(pointer(stFrameData), 5000)
            if ret != MV3D_RGBD_OK:
                print("[WARN] 获取帧失败")
                continue

            color_img = None
            depth_img = None

            for i in range(stFrameData.nImageCount):
                img_type = stFrameData.stImageData[i].enImageType
                width = stFrameData.stImageData[i].nWidth
                height = stFrameData.stImageData[i].nHeight
                data_len = stFrameData.stImageData[i].nDataLen
                img_data = string_at(stFrameData.stImageData[i].pData, data_len)

                if img_type == ImageType_Depth:
                    depth_values = struct.unpack('H' * (len(img_data) // 2), img_data)
                    depth_img = np.array(depth_values, dtype=np.uint16).reshape(height, width)

                if img_type == ImageType_RGB8_Planar:
                    plane_size = width * height
                    img_array = np.frombuffer(img_data, dtype=np.uint8)
                    if len(img_array) >= plane_size * 3:
                        r = img_array[0:plane_size].reshape(height, width)
                        g = img_array[plane_size:2 * plane_size].reshape(height, width)
                        b = img_array[2 * plane_size:3 * plane_size].reshape(height, width)
                        color_img = cv2.merge([b, g, r])
                        self.Handler.draw_lines(color_img)
                        self.Handler.detection_logic(depth_img)
                    else:
                        print("[WARN] 彩色图像数据长度异常")

            if color_img is not None:
                cv2.imshow("RGB Image", color_img)

            if depth_img is not None:
                depth_norm = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
                depth_colormap = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
                cv2.imshow("Depth Image", depth_colormap)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.save_depth_frame(depth_img)
            elif key == ord('q'):
                print("[INFO] 停止实时显示")
                break

        self.stop_flag = True
        cv2.destroyAllWindows()

    def start_streaming(self):
        """启动实时显示线程"""
        self.stop_flag = False
        t = threading.Thread(target=self.stream_frames, daemon=True)
        t.start()

    def stop_streaming(self):
        """停止取流"""
        self.stop_flag = True

    def close_camera(self):
        """关闭相机"""
        self.stop_flag = True
        time.sleep(0.5)
        if self.camera:
            self.camera.MV3D_RGBD_Stop()
            self.camera.MV3D_RGBD_CloseDevice()
            self.camera = None
            print("[INFO] 相机已关闭")


if __name__ == "__main__":
    Modbus = ModbusWriter(ip="0.0.0.0", port=502)
    Modbus.start_server()

    cam = CameraHandler()
    cam.open_camera()
    cam.start_streaming()

    while not cam.stop_flag:
        time.sleep(0.1)

    cam.close_camera()
