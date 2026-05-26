# 导入所需的库
import ctypes
import os  # 用于文件和目录操作
import threading
import time  # 用于时间相关操作
import math  # 用于数学计算
import struct  # 用于处理二进制数据与Python数据类型的转换

import torch
from watchdog.observers import Observer  # 用于监控文件系统变化
from watchdog.events import FileSystemEventHandler  # 用于处理文件系统事件
from ultralytics import YOLO  # 导入YOLO目标检测模型
import cv2  # 用于图像处理
from pymodbus.client import ModbusTcpClient  # 用于Modbus TCP通信
import numpy as np  # 用于数值计算

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

# ----------------- 配置参数区域 -----------------
# 监控的图片目录，新图片会被自动检测
WATCH_DIR = r'./camera/photo'
# 检测结果保存目录
SAVE_DIR = r'./camera/detect'
# YOLO模型权重文件路径
MODEL_PATH = r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\project\pillow4\weights\best.pt'

# 创建目录（如果不存在）
os.makedirs(WATCH_DIR, exist_ok=True)  # exist_ok=True表示目录存在也不报错
os.makedirs(SAVE_DIR, exist_ok=True)

# 物理尺寸参数（单位：毫米），根据实际场景调整
PHYS_W = 601  # 物理宽度
PHYS_H = 740  # 物理高度
Z_HEIGHT = 20.0  # Z轴高度（固定值）

# 机械臂Modbus通信配置
# ROBOT_IP = "192.168.0.30"  # 机械臂IP地址（备用）
ROBOT_IP = "169.254.0.66"  # 机械臂当前使用的IP地址
ROBOT_PORT = 502  # Modbus默认端口号
# 寄存器地址定义（与机械臂通信协议对应）
#REG_X = 0  # X坐标寄存器起始地址
#REG_Y = 2  # Y坐标寄存器起始地址
#REG_Z = 4  # Z坐标寄存器起始地址
#REG_A = 6  # 角度寄存器起始地址
#REG_zw = 8  # 动作信号寄存器起始地址

REG_X, REG_Y, REG_Z = 0, 2, 4
REG_SPEED, REG_RY, REG_A = 6, 8, 10
REG_zw, REG_10_FLAG, REG_11_FLAG, REG_12_FLAG = 12, 14, 16, 18

FLOAT_ORDER = "BADC"  # 浮点数在寄存器中的字节顺序

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


# ----------------- Modbus TCP通信相关函数 -----------------
def float_to_regs(value, order="BADC"):
    """
    将浮点数转换为Modbus寄存器值（两个16位整数）
    :param value: 要转换的浮点数
    :param order: 字节顺序
    :return: 转换后的两个寄存器值
    """
    # 将浮点数打包为4字节的二进制数据（大端模式）
    b = struct.pack('>f', value)
    # 定义不同字节顺序的映射关系
    byte_map = {
        "ABCD": [0, 1, 2, 3],
        "DCBA": [3, 2, 1, 0],
        "BADC": [1, 0, 3, 2],
        "CDAB": [2, 3, 0, 1]
    }
    # 按照指定顺序重新排列字节
    b_ordered = bytes([b[i] for i in byte_map[order]])
    # 将4字节数据解包为两个16位整数（寄存器值）
    return struct.unpack('>HH', b_ordered)


def send_position(client, x_mm, y_mm, z_mm, angle_deg, float_order=FLOAT_ORDER, pulse_zw=True):
    """
    发送坐标给机械臂，同时产生动作信号短脉冲
    :param client: Modbus客户端对象
    :param x_mm: X坐标（毫米）
    :param y_mm: Y坐标（毫米）
    :param z_mm: Z坐标（毫米）
    :param angle_deg: 角度（度）
    :param float_order: 浮点数字节顺序
    :param pulse_zw: 是否发送动作信号脉冲
    """
    # 将各坐标值转换为寄存器值
    x_regs = float_to_regs(x_mm, order=float_order)
    y_regs = float_to_regs(y_mm, order=float_order)
    z_regs = float_to_regs(z_mm, order=float_order)
    a_regs = float_to_regs(angle_deg, order=float_order)
    zw_regs_zero = float_to_regs(0.0, order=float_order)

    # 先写入坐标值和0值的动作信号
    client.write_registers(REG_X, list(x_regs))  # 写入X坐标寄存器
    client.write_registers(REG_Y, list(y_regs))  # 写入Y坐标寄存器
    client.write_registers(REG_Z, list(z_regs))  # 写入Z坐标寄存器
    client.write_registers(REG_A, list(a_regs))  # 写入角度寄存器
    client.write_registers(REG_zw, list(zw_regs_zero))  # 写入0值到动作信号寄存器

    if pulse_zw:
        # 发送动作信号短脉冲（置1）
        zw_regs_one = float_to_regs(1.0, order=float_order)
        client.write_registers(REG_zw, list(zw_regs_one))
        print(f"✅ 坐标已发送，REG_zw置1: X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f}, A={angle_deg:.1f}")
        time.sleep(0.1)  # 保持100ms的高电平
        # 恢复动作信号为0
        client.write_registers(REG_zw, list(zw_regs_zero))
        print("🔹 REG_zw已恢复为0")
    else:
        print(f"✅ 坐标已发送: X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f}, A={angle_deg:.1f}, REG_zw=0")


# ----------------- 加载YOLO模型 -----------------
model = YOLO(MODEL_PATH)  # 加载预训练的YOLO模型


# ----------------- 文件系统事件处理 -----------------
class NewImageHandler(FileSystemEventHandler):
    """处理新图片文件创建事件的类"""
    global Z_HEIGHT  # 引用全局变量Z_HEIGHT

    def __init__(self, modbus_client):
        """初始化方法"""
        super().__init__()  # 调用父类初始化方法
        self.client = modbus_client  # 保存Modbus客户端对象

    def on_created(self, event):
        """当有新文件创建时调用"""
        # 判断是否为图片文件（忽略目录和非图片文件）
        if not event.is_directory and event.src_path.lower().endswith(('.jpg', '.png', '.jpeg')):
            time.sleep(0.5)  # 等待文件完全写入
            print(f"\n🖼 检测新图片: {event.src_path}")
            self.run_yolo(event.src_path)  # 对新图片进行YOLO检测

    def detection_logic_cam1(img, display="Camera1"):  # 相机1的检测逻辑（基于质心跟踪）
        global tracker, targets  # 引用全局跟踪器和目标字典
        global t1, t2, delta_t, t_computed, prev_cy  # 引用时间相关全局变量
        global active_object, last_line1_time, last_line2_time, passed_line2  # 引用目标状态全局变量
        global current_trajectory, recording, trajectory_id  # （未定义，可能是遗留变量）
        global predict_buffer, model, scaler, model_device, inference_lock  # （部分未定义，推理相关）
        global cz_mm, Panduan, LAST_FLAG, delay_time, frame_counter  # 引用深度、状态、延迟等全局变量
        global depth_value, depth_ready, SPEED_NOW  # 引用深度和速度全局变量

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 将BGR格式转换为RGB格式（OpenCV默认BGR，此处可能冗余）
        now = time.time()  # 获取当前时间戳
        frame_counter += 1  # 帧计数器加1
        # active_object = False  # 注释：默认设置无活跃目标

        h, w = img.shape[:2]  # 获取图像的高度和宽度

        # ==== ROI 定义 ====
        roi_x_left = 20  # ROI左边界x坐标
        roi_x_right = 1660  # ROI右边界x坐标
        roi_w_px = roi_x_right - roi_x_left  # ROI宽度（像素）
        roi = img[:, roi_x_left:roi_x_right].copy()  # 截取ROI区域，深拷贝避免原图像修改

        # ✅✅ 相机1尺度 ✅✅
        real_width_cm = 60.09  # ROI对应的实际宽度（厘米）
        real_height_cm = 74.00  # 图像对应的实际高度（厘米）
        pixels_per_cm_x = roi_w_px / real_width_cm  # X方向每厘米对应的像素数
        pixels_per_cm_y = h / real_height_cm  # Y方向每厘米对应的像素数

        # ✅✅深度相机尺度✅✅
        pixels_per_mmx = 59.8 / 515 * 10  # X方向每毫米对应的像素数（根据深度相机校准）
        pixels_per_mmy = 50 / 435 * 10  # Y方向每毫米对应的像素数（根据深度相机校准）

        # ==== 坐标轴 ====
        cv2.line(img, (roi_x_right, 0), (0, 0), (0, 0, 0), 3)  # 绘制X轴基准线（顶部横线）
        for cm in range(0, int(real_width_cm) + 1, 5):  # 每隔5厘米绘制刻度
            x_pos = int(roi_x_right - cm * pixels_per_cm_x)  # 计算刻度的x坐标
            cv2.line(img, (x_pos, 0), (x_pos, 25), (0, 0, 0), 2)  # 绘制刻度线
            cv2.putText(img, f"{cm}", (x_pos - 25, 45),  # 绘制刻度值
                        cv2.FONT_HERSHEY_TRIPLEX, 1, (50, 50, 50), 2)  # 字体、大小、颜色、粗细

        cv2.line(img, (roi_x_right, 0), (roi_x_right, h), (0, 0, 0), 3)  # 绘制Y轴基准线（右侧竖线）
        for cm in range(0, int(real_height_cm) + 1, 5):  # 每隔5厘米绘制刻度
            y_pos = int(cm * pixels_per_cm_y)  # 计算刻度的y坐标
            cv2.line(img, (roi_x_right - 25, y_pos), (roi_x_right, y_pos), (0, 0, 0), 2)  # 绘制刻度线
            cv2.putText(img, f"{-cm}", (roi_x_right - 90, y_pos + 10),  # 绘制刻度值（负号表示方向）
                        cv2.FONT_HERSHEY_TRIPLEX, 1, (50, 50, 50), 2)  # 字体设置



        with inference_lock:  # 加推理锁，确保多线程下模型调用安全
            # 可选择先下采样 roi 再推理（见下一节），此处示范直接推理
            try:
                with torch.no_grad():  # 禁用梯度计算，节省内存并加速推理
                    results = model(roi, device="cuda", conf=0.8)  # 调用YOLO模型推理ROI区域，使用CUDA加速，置信度阈值0.8
            except Exception as e:  # 捕获推理异常
                print("[ERROR] 推理失败：", e)  # 打印错误信息
                return img  # 返回原图像

        r = results[0]  # 获取第一帧的推理结果

        if r.obb is not None and len(r.obb.xywhr) > 0:  # 若检测到旋转边界框（OBB）且数量大于0
            boxes = r.obb.xywhr.cpu().numpy()  # 获取OBB框坐标（中心x,y，宽w，高h，旋转角r），转CPU并转为numpy数组
            classes = r.obb.cls.cpu().numpy().astype(int)  # 获取类别ID，转CPU、numpy并转为整数
            confs = r.obb.conf.cpu().numpy()  # 获取置信度数组，转CPU并转为numpy数组

            for box, cls_id, conf in zip(boxes, classes, confs):  # 遍历每个检测框、类别ID和置信度
                # if conf < CONF_THRESH:
                #     continue  # 注释：置信度低于阈值时跳过（此处已在推理时设置conf=0.8，可省略）
                name = model.names[cls_id]  # 根据类别ID获取类别名称
                cx, cy, w, h, angle = box  # 解析OBB框参数

                angle_deg = math.degrees(angle)  # 将弧度转为角度
                # 保证以长边为参考
                if h > w:  # 若框的高度大于宽度（即长边为垂直方向）
                    angle_deg += 90.0  # 角度加90度，统一以长边为基准
                # 归一化到 [-180, 180)
                angle_deg = (angle_deg + 180) % 360 - 180  # 角度归一化处理
                # 压缩到 [-90, 90]
                if angle_deg > 90:  # 若角度大于90度
                    angle_deg -= 180  # 减180度
                elif angle_deg < -90:  # 若角度小于-90度
                    angle_deg += 180  # 加180度
                angle_deg = -angle_deg  # 角度取反（根据实际坐标系调整）

                print(
                    f"{name}: cx={cx:.1f}, cy={cy:.1f}, w={w:.1f}, h={h:.1f}, 长边角度={angle_deg:.1f}°,置信度={conf:.1f} ")  # 打印检测信息

                # 绘制质心和类别名
                cv2.circle(img, (int(cx), int(cy)), 6, (0, 0, 255), 2)  # 绘制质心（红色圆点，半径6，线宽2）
                cv2.putText(img, f"{name}", (int(cx) + 5, int(cy) - 5),  # 标注类别名
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)  # 绿色字体，大小1.0，线宽2

                rect = ((cx, cy), (w, h), math.degrees(angle))  # 构造旋转矩形参数：(中心点, 尺寸, 角度[度数])
                box_pts = cv2.boxPoints(rect)  # 计算旋转矩形的4个顶点坐标
                box_pts = np.int0(box_pts)  # 转换为整数坐标
                cv2.drawContours(img, [box_pts], 0, (0, 0, 255), 2)  # 绘制旋转边界框（红色，线宽2）

                # === 像素->毫米 ===
                cx_mm = -(cy / pixels_per_cm_y) * 10  # 将Y方向像素坐标转为毫米（负号调整方向）
                cy_mm = (roi_x_right - cx) / pixels_per_cm_x * 10  # 将X方向像素坐标转为毫米

                shendu_location = (
                1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))  # 计算深度相机的查询坐标
                # === 初始化 targets ===
                if targets is None:  # 若目标字典未初始化
                    targets = {  # 初始化目标字典
                        "last_line1_time": None,  # 上一次经过第一条线的时间
                        "last_line2_time": None,  # 上一次经过第二条线的时间
                        "frame_count": 0,  # ✅ 帧计数器（记录目标被检测的帧数）
                        "trajectory": [],  # 目标轨迹
                    }

                # === 更新帧计数 ===
                targets["frame_count"] += 1  # 目标的检测帧数加1

                # === 仅在第10帧调用一次深度线程 ===
                # now1 = time.time()
                # if targets["frame_count"] == 10:
                #     shendu_location = (1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))
                #     threading.Thread(
                #         target=async_get_depth,
                #         args=(shendu_location[0], shendu_location[1]),
                #         daemon=True
                #     ).start()
                #
                # with depth_lock:
                #     if depth_ready:
                #         cz_mm = depth_value
                #         now2 = time.time()
                #         print("[INFO] 第10帧，已触发一次深度获取", shendu_location, depth_value)
                #         print("高度获取花费时间：", now2 - now1)
                #         depth_ready = False
                if targets[
                    "frame_count"] == 10:  # 若目标被检测到第10帧-------------------------------------------------------------------------------------------------------------------------------
                    # now1 = time.time()
                    shendu_location = (
                    1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))  # 计算深度查询坐标
                    cz_mm = cam.get_height(shendu_location[0], shendu_location[1])  # 同步调用深度相机获取高度
                    # now2 = time.time()
                    cz_mm = 1190 - cz_mm  # 深度值校准（根据实际场景调整）
                    print("[INFO] 第10帧，已触发深度获取", shendu_location, cz_mm)  # 打印深度信息
                # print("高度获取花费时间：", now2 - now1)

                label = f"({cx_mm:.1f}mm, {cy_mm:.1f}mm, {cz_mm:.1f}mm)"  # 构造坐标标签（x,y,z毫米）
                cv2.putText(img, label, (int(cx + 5), int(cy + 40)),  # 标注坐标
                            cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 2)  # 绿色字体
        else:  # 未检测到OBB
            print("No OBB detected")  # 打印提示

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
                    "serial": bytes(info.SpecialInfo.stGigEInfo.chSerialNumber).decode('ascii',
                                                                                       errors='ignore').strip(),
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

    def run_yolo(self, img_path):
        """运行YOLO模型进行目标检测"""
        # 执行目标检测（obb表示定向边界框检测，适用于旋转目标）
        results = model.predict(source=img_path, task="obb", save=False)
        # 获取原图尺寸
        img_h, img_w = results[0].orig_shape[:2]
        # 复制原图用于绘制检测结果
        img_plot = results[0].orig_img.copy()

        # 绘制自定义坐标轴（便于理解坐标转换）
        origin = (img_w - 1, 0)  # 原点位置
        # 绘制X轴（红色）
        cv2.line(img_plot, origin, (img_w - 1, img_h - 1), (0, 0, 255), 2)
        # 绘制Y轴（蓝色）
        cv2.line(img_plot, origin, (0, 0), (255, 0, 0), 2)
        # 添加轴标签
        cv2.putText(img_plot, 'X', (img_w - 15, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img_plot, 'Y', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        any_detected = False  # 标记是否检测到目标

        # 处理检测结果
        for r in results:
            # 检查是否有定向边界框检测结果
            if r.obb is not None and len(r.obb.xywhr) > 0:
                any_detected = True  # 标记已检测到目标
                # 遍历每个检测框
                for idx, box in enumerate(r.obb.xywhr.cpu().numpy(), start=1):
                    # 解析检测框参数：中心x、中心y、宽度、高度、旋转角度（弧度）
                    cx, cy, w, h, angle = box
                    # 将图像坐标转换为物理坐标（毫米）
                    phys_x = (0 - cy / img_h) * PHYS_H
                    phys_y = ((img_w - cx) / img_w) * PHYS_W - 710  # -710是偏移校准值

                    # 计算旋转矩形的四个顶点（用于绘制）
                    pts = cv2.boxPoints(((cx, cy), (w, h), math.degrees(angle)))
                    pts = pts.astype(int)  # 转换为整数坐标
                    # 绘制检测框（绿色）
                    cv2.drawContours(img_plot, [pts], 0, (0, 255, 0), 2)
                    # 绘制中心点（红色）
                    cv2.circle(img_plot, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    # 标记检测框序号
                    cv2.putText(img_plot, f"#{idx}", (int(cx) + 5, int(cy) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # 绘制YOLO水平线（青色）
                    cv2.line(img_plot, (0, int(cy)), (img_w - 1, int(cy)), (255, 255, 0), 1, lineType=cv2.LINE_AA)
                    cv2.putText(img_plot, 'YOLO Horizontal', (10, int(cy) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                    # 计算长边角度（更准确的目标朝向）
                    # 获取四个边的端点
                    edges = [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
                    # 计算每条边的长度
                    edge_lengths = [np.linalg.norm(np.array(p2) - np.array(p1)) for p1, p2 in edges]
                    # 找到最长边的索引
                    longest_idx = np.argmax(edge_lengths)
                    p1, p2 = edges[longest_idx]  # 最长边的两个端点
                    # 计算长边的方向向量
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    # 计算角度（弧度转角度）
                    final_angle = math.degrees(math.atan2(dy, dx))

                    # 角度调整（确保在0-180度范围内）
                    if final_angle < 0:
                        final_angle = 180 + final_angle

                    # 绘制最长边（黄色）
                    cv2.line(img_plot, tuple(p1), tuple(p2), (0, 255, 255), 2)
                    # 显示角度值
                    cv2.putText(img_plot, f"Angle: {final_angle:.1f}", (int(cx), int(cy) - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    # 发送坐标给机械臂
                    send_position(self.client, phys_x, phys_y, Z_HEIGHT, final_angle, pulse_zw=True)
                    print(f"⏱ 检测框 {idx} 坐标已发送，等待 20 秒再发送下一个目标...")
                    time.sleep(20)  # 测试用20秒，正式环境可改为300秒

        # 如果没有检测到任何目标，发送0值保持REG_zw为0
        if not any_detected:
            send_position(self.client, 0, 0, 0, 0, pulse_zw=False)

        # 保存检测结果图片
        filename = os.path.basename(img_path)  # 获取原文件名
        save_path = os.path.join(SAVE_DIR, f"detect_{filename}")  # 构造保存路径
        cv2.imwrite(save_path, img_plot)  # 保存图片
        print(f"✅ 检测结果已保存: {save_path}")


# ----------------- 主程序入口 -----------------
if __name__ == "__main__":
    # 创建Modbus TCP客户端并连接机械臂
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT)
    if not client.connect():
        print("❌ 连接机械臂失败")
        exit()  # 连接失败则退出程序
    print("✅ 成功连接机械臂")

    # 创建事件处理器（传入Modbus客户端）
    event_handler = NewImageHandler(client)
    # 创建文件系统监控器
    observer = Observer()
    # 配置监控器：监控WATCH_DIR目录，不递归子目录
    observer.schedule(event_handler, WATCH_DIR, recursive=False)
    # 启动监控器
    observer.start()
    print(f"📂 监听目录: {WATCH_DIR}，有新图片将自动检测并发送机械臂坐标...")

    try:
        # 保持程序运行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断信号，停止监控器
        observer.stop()
    # 等待监控器线程结束
    observer.join()
    # 关闭Modbus连接
    client.close()