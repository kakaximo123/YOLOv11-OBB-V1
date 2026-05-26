# dual_cam_pyqt_full.py
import sys, threading, math, struct, ctypes, numpy as np, cv2
import os
import csv
from ctypes import byref, cast, POINTER
from pymodbus.client import ModbusTcpClient
import socket
import time
from collections import deque
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import joblib
import torch
from chen.Python import get_self
from ultralytics import YOLO
from chen.Python.get_height1 import CameraHandler
from sort import Sort  # 引入 SORT
model = YOLO(r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\runs\obb\train19\weights\best.pt')
tracker = Sort(max_age=5, min_hits=1, iou_threshold=0.3)  # 初始化 SORT
targets = None  # key: target_id, value: {"last_line1_time":..., "last_line2_time":..., "trajectory":[...]}
try:
    from MvCameraControl_class import *
except Exception as e:
    print("无法导入 MvCameraControl_class，请检查 SDK")
    sys.exit(1)

# ================== Modbus 配置 ==================
ROBOT_IP, ROBOT_PORT = "169.254.0.66", 502
REG_X, REG_Y, REG_Z = 0, 2, 4
REG_SPEED, REG_RY, REG_RZ = 6, 8, 10
REG_9_FLAG, REG_10_FLAG, REG_11_FLAG, REG_12_FLAG = 12, 14, 16, 18

modbus_lock = threading.Lock()
modbus_client = None

# ================== 相机 IP ==================
CAM1_IP = "169.254.0.102"
CAM2_IP = "169.254.0.101"

# ================== 检测时间设置 ==================
prev_roi_gray = None
global_t = None  # cam1 计算的时间差 t
t1, t2 = None, None  # 时间戳
t_computed = False
delta_t = None
LAST_FLAG1 = True
prev_cy = None  # 新增：全局记录上一次质心 y
# 在文件开头全局变量处加：
last_line1_time = None
last_line2_time = None
active_object = False
DEBOUNCE_INTERVAL = 0.5  # 去抖动时间（秒）
passed_line2 = False  # 记录物体是否已经过了L2
TIMEOUT_LINE2 = 10

LAST_FLAG = False
# ========== 推理相关全局变量 ==========
inference_lock = threading.Lock()  # 推理锁，保证线程安全

stop_event = threading.Event()
frame_lock = threading.Lock()

cz_mm = 20
Panduan = 0
delay_time = 0
frame_counter = 0
depth_lock = threading.Lock()
depth_value = 0
depth_ready = 0
SPEED_NOW = 0
dot_list = []
object_line = {"line0": False, "line1": False, "line2": False}
time_dig = [0, 0, 0]
object_count = 0
# ================== 工具函数 ==================
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


def float_to_regs(value, order="BADC"):
    b = struct.pack('>f', float(value))
    byte_map = {"ABCD": [0, 1, 2, 3], "DCBA": [3, 2, 1, 0], "BADC": [1, 0, 3, 2], "CDAB": [2, 3, 0, 1]}
    b_ordered = bytes([b[i] for i in byte_map[order]])
    return struct.unpack('>HH', b_ordered)


def send_position_6axis(x, y, z, rs, ry, rz, float_order="BADC"):
    regs_start = [REG_X, REG_Y, REG_Z, REG_SPEED, REG_RY, REG_RZ]
    values = [x, y, z, rs, ry, rz,]
    for value, reg in zip(values, regs_start):
        regs = float_to_regs(value, order=float_order)
        with modbus_lock:
            modbus_client.write_registers(reg, list(regs))


def send_all_flag(flag9, flag10, flag11, flag12, float_order="BADC"):
    regs_start = [REG_9_FLAG, REG_10_FLAG, REG_11_FLAG, REG_12_FLAG]
    values = [flag9, flag10, flag11, flag12]
    for value, reg in zip(values, regs_start):
        regs = float_to_regs(value, order=float_order)
        with modbus_lock:
            modbus_client.write_registers(reg, list(regs))


# ================== 相机检测辅助 ==================
def list_all_cameras():
    device_list = MV_CC_DEVICE_INFO_LIST()
    ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)
    if ret != 0:
        print("枚举相机失败")
        return []

    cameras = []
    print(f"检测到 {device_list.nDeviceNum - 1} 台相机：")
    for i in range(device_list.nDeviceNum):
        info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents

        if info.nTLayerType == MV_GIGE_DEVICE:
            # 解析 IP
            nIp1 = (info.SpecialInfo.stGigEInfo.nCurrentIp >> 24) & 0xFF
            nIp2 = (info.SpecialInfo.stGigEInfo.nCurrentIp >> 16) & 0xFF
            nIp3 = (info.SpecialInfo.stGigEInfo.nCurrentIp >> 8) & 0xFF
            nIp4 = info.SpecialInfo.stGigEInfo.nCurrentIp & 0xFF
            ip_str = f"{nIp1}.{nIp2}.{nIp3}.{nIp4}"

            cam_info = {
                "ip": ip_str,
                "model": bytes(info.SpecialInfo.stGigEInfo.chModelName).decode('ascii', errors='ignore').strip(),
                "serial": bytes(info.SpecialInfo.stGigEInfo.chSerialNumber).decode('ascii', errors='ignore').strip(),
                "info": info
            }
            cameras.append(cam_info)
            print(f"[{i}] IP: {cam_info['ip']}, 型号: {cam_info['model']}, SN: {cam_info['serial']}")

    return cameras


def find_device_by_ip(ip):
    cams = list_all_cameras()
    for cam in cams:
        if cam["ip"] == ip:
            return cam["info"]
    return None


def async_get_depth(sx, sy):
    """子线程获取深度，避免阻塞主线程"""
    global depth_value, depth_ready
    try:
        cz = cam.get_height(int(sx), int(sy))
        cz = 1195 - cz
        with depth_lock:
            depth_value = cz
            depth_ready = True
    except Exception as e:
        print("[ERROR] 深度获取失败:", e)

# ================== cam1 检测逻辑（质心） ==================
def detection_logic_cam1(img, display="Camera1"):
    global tracker, targets
    global t1, t2, delta_t, t_computed, prev_cy
    global active_object, last_line1_time, last_line2_time, passed_line2
    global current_trajectory, recording, trajectory_id
    global predict_buffer, model, scaler, model_device, inference_lock
    global cz_mm, Panduan, LAST_FLAG, delay_time, frame_counter
    global depth_value, depth_ready, SPEED_NOW, LAST_FLAG1
    global object_line, time_dig, dot_list, object_count

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    now = time.time()
    frame_counter += 1
    # active_object = False
    # 每隔一帧推理一次
    # if frame_counter % 2 != 0:
    #     return img  # ✅ 跳过推理，直接返回

    h, w = img.shape[:2]


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
    pixels_per_cm_y = h / real_height_cm

    # ✅✅深度相机尺度✅✅
    pixels_per_mmx = 59.8 / 515 * 10
    pixels_per_mmy = 50 / 435 * 10

    # ==== 坐标轴 ====
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

    # 绘制判断线
    # ==== 两条检测线 ====🩷🩷
    trigger_line = {"bottom_y": 2 * h // 3, "middle_y": 2 * h // 5, "top_y": h // 5}
    cv2.line(img, (0, trigger_line["bottom_y"]), (roi_x_right, trigger_line["bottom_y"]), (255, 0, 0), 2)
    cv2.putText(img, "bottom_y", (10, trigger_line["bottom_y"] - 10),
                cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 2)

    cv2.line(img, (0, trigger_line["top_y"]), (roi_x_right, trigger_line["top_y"]), (255, 0, 0), 2)
    cv2.putText(img, "top_y", (10, trigger_line["top_y"] - 10),
                cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 2)

    with inference_lock:
        # 可选择先下采样 roi 再推理（见下一节），此处示范直接推理
        try:
            with torch.no_grad():
                results = model(roi, device="cuda", conf=0.6, verbose=False)  # 直接给 conf 门限，避免后面大量筛选
        except Exception as e:
            print("[ERROR] 推理失败：", e)
            return img

    r = results[0]



    if r.obb is not None and len(r.obb.xywhr) > 0:
        boxes = r.obb.xywhr.cpu().numpy()
        classes = r.obb.cls.cpu().numpy().astype(int)
        confs = r.obb.conf.cpu().numpy()  # 置信度数组

        for box, cls_id, conf in zip(boxes, classes, confs):
            # if conf < CONF_THRESH:
            #     continue  # 置信度低于阈值，跳过
            name = model.names[cls_id]  # 类别名
            cx, cy, w, h, angle = box

            angle_deg = math.degrees(angle)
            # 保证以长边为参考
            if h > w:
                angle_deg += 90.0
            # 归一化到 [-180, 180)
            angle_deg = (angle_deg + 180) % 360 - 180
            # 压缩到 [-90, 90]
            if angle_deg > 90:
                angle_deg -= 180
            elif angle_deg < -90:
                angle_deg += 180
            angle_deg = -angle_deg

            print(f"{name}: cx={cx:.1f}, cy={cy:.1f}, w={w:.1f}, h={h:.1f}, 长边角度={angle_deg:.1f}°,置信度={conf:.1f} ")



            # 绘制质心和ID
            cv2.circle(img, (int(cx), int(cy)), 6, (0, 0, 255), 2)
            cv2.putText(img, f"{name}", (int(cx) + 5, int(cy) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            rect = ((cx, cy), (w, h), math.degrees(angle))  # (中心点, 尺寸, 角度[度数])
            box_pts = cv2.boxPoints(rect)  # 得到 4 个顶点
            box_pts = np.int0(box_pts)  # 转换为整数
            cv2.drawContours(img, [box_pts], 0, (0, 0, 255), 2)

            # === 像素->mm ===
            cx_mm = -(cy / pixels_per_cm_y) * 10
            cy_mm = (roi_x_right - cx) / pixels_per_cm_x * 10

            dot_list.append((cy, now))
            if len(dot_list) > 30:
                dot_list.pop(0)



            # shendu_location = (1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))
            # === 初始化 targets ===
            if targets is None:
                targets = {
                    "last_line1_time": None,
                    "last_line2_time": None,
                    "last_line1_flag": None,
                    "last_line2_flag": None,
                    "frame_count": 0,  # ✅ 帧计数器
                    "trajectory": 0,
                }

            # === 更新帧计数 ===
            targets["frame_count"] += 1



            if targets["frame_count"] == 6:
                object_count += 1
                # now1 = time.time()

                shendu_location = (1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))





                # cz_mm = cam.get_height(shendu_location[0], shendu_location[1])
                cam.save_depth_frame(object_count)
                # now2 = time.time()
                depth = np.load(f"D:\examples_project\python projects\depth\depth_{object_count}.npy")
                depth_value = depth[shendu_location[0] + 4, shendu_location[1] + 4]  # 获取任意像素点深度
                cz_mm = 1198 - depth_value
                print("[INFO] 第10帧，已触发深度获取", shendu_location, cz_mm)

            # print("高度获取花费时间：", now2 - now1)

            label = f"({cx_mm:.1f}mm, {cy_mm:.1f}mm, {cz_mm:.1f}mm)"
            cv2.putText(img, label, (int(cx + 5), int(cy + 40)),
                        cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 2)
            # bottom和middle线之间


            if cy > trigger_line["bottom_y"]:
                object_line["line0"] = True
                time_dig[0] = now
                send_all_flag(0, 0, 0, 0, float_order="BADC")
                # send_position_6axis(0, 0, 0, 0, 0, 0, float_order="BADC")
                print(f"检测到物体")

            if trigger_line["top_y"] <= cy <= trigger_line["bottom_y"] and object_line["line0"]:
                object_line["line1"] = True
                time_dig[1] = now
                if targets["last_line1_flag"] is None:
                    targets["last_line1_time"] = now
                    targets["last_line1_flag"] = True
                send_all_flag(1, 0, 0, 0, float_order="BADC")
                if cz_mm > 300 or cz_mm < 5:
                    cz_mm = 40
                send_position_6axis(cx_mm, cy_mm, cz_mm / 2, 0, 0, -angle_deg, float_order="BADC")

                print(f"发送跟随坐标，坐标信息为(中心x，中心y，高度，角度), {cx_mm, cy_mm, cz_mm, angle_deg}")

            if cy < trigger_line["top_y"] and object_line["line1"]:
                object_line["line2"] = True

                time_dig[2] = now
                # 当数据量够时计算速度
                if len(dot_list) >= 2:
                    cy_vals = [item[0] for item in dot_list]
                    t_vals = [item[1] for item in dot_list]
                    cy_max, cy_min = max(cy_vals), min(cy_vals)
                    t_max = t_vals[cy_vals.index(cy_max)]
                    t_min = t_vals[cy_vals.index(cy_min)]
                    delta_cy = cy_max - cy_min
                    delta_t = abs(t_max - t_min)
                    print(dot_list)

                    if delta_t > 0:
                        SPEED_NOW = (delta_cy / pixels_per_cm_y) / delta_t   # 单位：像素/秒
                        print("速度:", round(SPEED_NOW * 10, 2))
                    # else:
                    #     SPEED_NOW = 0.0
                if targets["last_line2_flag"] is None:
                    targets["last_line2_flag"] = True

                    for i in range(1, 5):
                        send_all_flag(1, 1, 0, 1, float_order="BADC")
                    targets["last_line2_time"] = now
                    time_interval = targets["last_line2_time"] - targets["last_line1_time"]
                    # distance = (2/3 - 1/5) * real_height_cm
                    # SPEED_NOW = distance/time_interval
                    send_position_6axis(cx_mm, cy_mm, cz_mm, 0, round(SPEED_NOW * 10, 2), -angle_deg, float_order="BADC")
                    delay_time = 57 / SPEED_NOW
                    print("速度, 触发等待时间", round(SPEED_NOW * 10, 2), delay_time)
                    # print("时间间隔, 距离, 速度, 触发等待时间", time_interval, distance, SPEED_NOW, delay_time)

    else:
        print("No OBB detected")

    if object_line["line2"]:
        if SPEED_NOW < 25:
            delay_time = delay_time
        elif 25 <= SPEED_NOW < 45:
            delay_time -= 0.01
        elif 45 <= SPEED_NOW < 70:
            delay_time -= 0.6
        elif 70 <= SPEED_NOW:
            delay_time -= 1
        if now - targets["last_line2_time"] > 0.5:
            if LAST_FLAG1:
                send_all_flag(0, 0, 0, 0, float_order="BADC")
                LAST_FLAG1 = False
        if now - targets["last_line2_time"] > delay_time:
            for i in range(1, 20, 1):
                print(f"开始跟随,物体{object_count}")
            send_all_flag(0, 0, 1, 0, float_order="BADC")
            for key in object_line:
                object_line[key] = False
            targets = None
            LAST_FLAG1 = True
            cz_mm = 20
            delay_time = 0
            SPEED_NOW = 0
    if object_line["line1"] and object_line["line2"] is False:
        if now - time_dig[1] > 15:
            for key in object_line:
                object_line[key] = False
            targets = None
            LAST_FLAG1 = True
            cz_mm = 20
            delay_time = 0
            SPEED_NOW = 0
            dot_list.clear()
            send_all_flag(0, 0, 0, 0, float_order="BADC")
            send_position_6axis(0, 0, 0, 0, 0, 0, float_order="BADC")
            for i in range(100):
                print("出现异常，进行清零操作")

    return img

# ================== cam2 检测逻辑（矩形边框） ==================
def detection_logic_cam2(img):
    global prev_roi_gray, delta_t
    # 图像尺寸
    h, w = img.shape[:2]
    roi_w = w  # ROI=整幅图

    # 提取整幅图像
    roi = img.copy()

    # ===== 根据 delta_t 选择基准线 =====
    if delta_t is None:
        delta_t = 1

    if 0 < delta_t < 0.43:
        claw_line_y = h - 120
        line_label = "Claw Line 1"
    elif 0.43 <= delta_t < 0.58:
        claw_line_y = h - 320
        line_label = "Claw Line 2"
    elif 0.48 <= delta_t < 0.55:
        claw_line_y = h - 520
        line_label = "Claw Line 2"
    elif 0.55 <= delta_t < 0.7:
        claw_line_y = h - 660
        line_label = "Claw Line 2"
    elif 0.7 <= delta_t < 0.75:
        claw_line_y = h - 700
        line_label = "Claw Line 3"
    elif 0.75 <= delta_t < 0.85:
        claw_line_y = h - 750
        line_label = "Claw Line 3"
    else:
        claw_line_y = h - 800
        line_label = "Claw Line 4"

    # 实际尺寸参数
    real_width_cm = 60.59
    real_height_cm = 61.28
    pixels_per_cm_x = roi_w / real_width_cm
    pixels_per_cm_y = h / real_height_cm

    # ======= 绘制坐标轴 =======
    # Y轴
    cv2.line(img, (0, 0), (0, h), (0, 0, 0), 2)
    for cm in range(0, int(real_height_cm) + 1, 5):
        y_pos = int(cm * pixels_per_cm_y)
        cv2.line(img, (0, y_pos), (15, y_pos), (0, 0, 0), 2)
        cv2.putText(img, f"{cm}", (20, y_pos + 5),
                    cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1)

    # X轴
    cv2.line(img, (0, h), (roi_w, h), (0, 0, 0), 2)
    for cm in range(0, int(real_width_cm) + 1, 5):
        x_pos = int(cm * pixels_per_cm_x)
        cv2.line(img, (x_pos, h - 15), (x_pos, h), (0, 0, 0), 2)
        cv2.putText(img, f"{cm}", (x_pos - 10, h - 20),
                    cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1)

    # ======= 绘制基准线 =======
    baseline_y = h - 5

    cv2.line(img, (0, baseline_y), (roi_w, baseline_y), (220, 245, 245), 2)
    cv2.putText(img, "Horizontal baseline", (10, baseline_y - 10),
                cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)

    cv2.line(img, (0, h - 800), (roi_w, h - 800), (0, 215, 255), 2)
    cv2.putText(img, "Claw Line 4", (10, h - 810),
                cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)

    cv2.line(img, (0, h - 700), (roi_w, h - 700), (0, 215, 255), 2)
    cv2.putText(img, "Claw Line 3"
                     "", (10, h - 710),
                cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)

    cv2.line(img, (0, h - 620), (roi_w, h - 620), (0, 215, 255), 2)
    cv2.putText(img, "Claw Line 2 ", (10, h - 630),
                cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)

    cv2.line(img, (0, h - 450), (roi_w, h - 450), (0, 215, 255), 2)
    cv2.putText(img, "Claw Line 1 ", (10, h - 460),
                cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)

    # ======= 边缘检测与轮廓 =======
    # 定义一个窄矩形（基于爪子基准线，向上取5像素）
    roi_height = 10  # 检测带高度
    y1 = max(claw_line_y - roi_height // 2, 0)
    y2 = min(claw_line_y + roi_height // 2, h)
    roi_band = roi[y1:y2, :]

    # 转灰度
    roi_gray = cv2.cvtColor(roi_band, cv2.COLOR_BGR2GRAY)

    # # 使用全局变量保存上一帧
    global prev_roi_gray

    if 'prev_roi_gray' in globals() and prev_roi_gray is not None:
        diff = cv2.absdiff(roi_gray, prev_roi_gray)
        diff = cv2.GaussianBlur(diff, (3, 3), 0)
        _, thresh = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)

        # 计算变化比例
        change_ratio = np.sum(thresh > 0) / (roi_band.shape[0] * roi_band.shape[1])
        # send_detect_flag(1)
        # send_all_flag(0, 0, 0)
        if change_ratio > 0.01:  # 超过8%像素变化
            print("000000")
            # send_all_flag(0, 0, 1)
            # time.sleep(0.2)

    prev_roi_gray = roi_gray.copy()

    # 在画面上画出检测矩形
    cv2.rectangle(img, (0, y1), (roi_w, y2), (0, 0, 255), 2)

    return img


# =============== 相机线程函数（按 IP） ===============
def camera_worker(ip, mode, frame_dict, key):
    """
    更稳健的 camera_worker：
    - 重连逻辑：只有在需要重连时才枚举设备
    - finally 中确保释放资源
    - 响应 stop_event 以优雅退出
    """
    while not stop_event.is_set():
        cam = None
        sel_info = None
        try:
            # 尝试找到设备（重连循环）
            sel_info = find_device_by_ip(ip)
            if sel_info is None:
                print(f"[{mode}] 未找到相机 IP={ip}，等待重试...")
                # 响应退出信号
                for _ in range(4):
                    if stop_event.is_set():
                        return
                    time.sleep(0.5)
                continue

            cam = MvCamera()
            cam.MV_CC_CreateHandle(sel_info)
            cam.MV_CC_OpenDevice()

            # 获取 payload 等信息
            stParam = MVCC_INTVALUE()
            cam.MV_CC_GetIntValue("PayloadSize", stParam)
            payload_size = stParam.nCurValue
            data_buf = (ctypes.c_ubyte * payload_size)()
            frame_info = MV_FRAME_OUT_INFO_EX()

            cam.MV_CC_StartGrabbing()
            print(f"[{mode}] 已连接相机 IP={ip}")

            last_frame_time = time.time()
            # 拉帧循环
            while not stop_event.is_set():
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

                    # 处理
                    if mode == "cam1":
                        img_out = detection_logic_cam1(img)
                    else:
                        img_out = detection_logic_cam2(img)

                    # 写入共享字典时加锁
                    with frame_lock:
                        frame_dict[key] = img_out

                    last_frame_time = time.time()
                else:
                    # 没拿到帧，检查超时并重连
                    if time.time() - last_frame_time > 5:
                        print(f"[{mode}] 超时,准备重连...")
                        break
                    # 轻微等待，继续尝试
                    if stop_event.wait(0.01):
                        break

        except Exception as e:
            print(f"[{mode}] 相机异常: {e}")

        finally:
            # 确保释放资源（无论异常或正常退出）
            try:
                if cam is not None:
                    try:
                        cam.MV_CC_StopGrabbing()
                    except Exception:
                        pass
                    try:
                        cam.MV_CC_CloseDevice()
                    except Exception:
                        pass
                    try:
                        cam.MV_CC_DestroyHandle()
                    except Exception:
                        pass
                print(f"[{mode}] 资源已释放")
            except Exception as e:
                print(f"[{mode}] 释放相机异常：{e}")
            # 给 SDK 一点时间彻底清理，避免下次立即打开卡住
            for _ in range(5):
                if stop_event.is_set():
                    break
                time.sleep(0.1)
            # 重连前短暂等待，避免 tight-loop
            if not stop_event.is_set():
                time.sleep(0.5)


# =============== PyQt 界面类 ===============
class DualCameraUI(QWidget):
    def __init__(self, frame_dict):
        super().__init__()
        self.setWindowTitle("Dual Camera PyQt View")
        self.frame_dict = frame_dict

        self.label1 = QLabel("Camera1");
        self.label1.setAlignment(Qt.AlignCenter)
        self.label2 = QLabel("Camera2");
        self.label2.setAlignment(Qt.AlignCenter)

        hbox = QHBoxLayout();
        hbox.addWidget(self.label1);
        hbox.addWidget(self.label2)
        self.setLayout(hbox)

        self.timer = QTimer(self);
        self.timer.timeout.connect(self.update_frames)
        self.timer.start(30)

    def update_frames(self):
        for key, label in [("cam1", self.label1), ("cam2", self.label2)]:
            frame = self.frame_dict.get(key)
            if frame is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
                label.setPixmap(QPixmap.fromImage(qimg).scaled(
                    label.width() if label.width() > 0 else w,
                    label.height() if label.height() > 0 else h,
                    Qt.KeepAspectRatio))


# =============== 主入口 ===============
def main():
    global modbus_client
    modbus_client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT)
    modbus_client.connect()


    frame_dict = {"cam1": None, "cam2": None}

    # 启动两个相机线程（按 IP）
    t_cam1 = threading.Thread(target=camera_worker, args=(CAM1_IP, "cam1", frame_dict, "cam1"), daemon=True)
    # t_cam2 = threading.Thread(target=camera_worker, args=(CAM2_IP, "cam2", frame_dict, "cam2"), daemon=True)

    t_cam1.start()
    # t_cam2.start()

    app = QApplication(sys.argv)
    ui = DualCameraUI(frame_dict)
    ui.show()

    # 在程序退出前，设置 stop_event 并等待线程退出
    def on_exit():
        # folder = r"D:\examples_project\python projects\depth"  # 文件夹路径
        # for file in os.listdir(folder):
        #     if file.endswith(".npy"):
        #         os.remove(os.path.join(folder, file))
        # print("所有 .npy 文件已删除")

        print("[Main] 收到退出信号，停止线程...")
        stop_event.set()
        # 等待线程短时间退出（daemon 线程会随进程退出，但我们尽量优雅）
        t_cam1.join(timeout=2)
        # t_cam2.join(timeout=2)
        cam.close_camera()
        send_all_flag(0, 0, 0, 0, float_order="BADC")
        send_position_6axis(0, 0, 0, 0, 0, 0, float_order="BADC")
        print("[Main] 线程已停止，退出程序。")

    app.aboutToQuit.connect(on_exit)
    sys.exit(app.exec_())


if __name__ == "__main__":
    cam = CameraHandler()
    cam.open_camera()


    main()
