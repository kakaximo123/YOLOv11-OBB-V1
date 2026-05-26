#ByteTrack算法  抓ID最小与数据库联通

# -*- coding: utf-8 -*-
import os
import sys
import ctypes
import base64
import threading
import time
import struct
import math
import numpy as np
import cv2
import torch
import threading
import queue
import time
import logging
import random
import json
import asyncio
import websockets
from dataclasses import dataclass
from typing import Optional, Dict
import base64
from ctypes import byref, pointer, string_at

# --------------------------
# 路径 & DLL 配置
# --------------------------
ultralytics_path = r"D:\examples_project\python projects\ultralytics-main"
if ultralytics_path not in sys.path:
    sys.path.insert(0, ultralytics_path)

dll_dir = r"D:\Conda\envs\pytorch\Library\bin"
if os.path.exists(dll_dir):
    os.add_dll_directory(dll_dir)
    print(f"[INFO] 已添加 PyTorch DLL 目录: {dll_dir}")
else:
    print(f"[ERROR] DLL 目录不存在: {dll_dir}")

# --------------------------
# 导入 SDK & 库
# --------------------------
from Mv3dRgbdImport.Mv3dRgbdDefine import *
from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import (
    DeviceType_Ethernet, DeviceType_USB,
    DeviceType_Ethernet_Vir, DeviceType_USB_Vir,
    MV3D_RGBD_OK, ImageType_Depth, ImageType_RGB8_Planar, ImageType_Rgbd
)
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext

# ultralytics YOLO（本地源码优先）
from ultralytics import YOLO
import requests
import json

# 兼容旧代码：Sort
from sort import Sort


# from bytetracker import BYTETracker
ImageType_Color = ImageType_RGB8_Planar

# 请确认权重路径正确
Model = YOLO(r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\runs\obb\train25\weights\best.pt')


# ================== [新增] 1. WebSocket 配置模块 ==================
@dataclass
class Config:
    websocket_uri: str = "ws://127.0.0.1:8765/robot"
    queue_maxsize: int = 3  # 队列小一点，延迟低
    jpeg_quality: int = 60


@dataclass
class FramePackage:
    image_bytes: bytes
    telemetry: Dict


# ================== [新增] 2. WebSocket 发送类 ==================
class VideoSender:
    def __init__(self, config: Config):
        self.config = config
        self.send_queue = queue.Queue(maxsize=config.queue_maxsize)
        self.stop_event = threading.Event()
        self.send_thread = None

    async def _async_send_loop(self):
        logging.info(f"🔌 正在连接服务器: {self.config.websocket_uri}")
        while not self.stop_event.is_set():
            try:
                async with websockets.connect(self.config.websocket_uri, ping_interval=None) as ws:
                    logging.info("✅ WebSocket 连接成功")
                    while not self.stop_event.is_set():
                        try:
                            # 0.05秒超时，为了能响应退出信号
                            package = self.send_queue.get(timeout=0.05)
                            await ws.send(json.dumps(package.telemetry))
                            await ws.send(package.image_bytes)
                        except queue.Empty:
                            continue
                        except Exception:
                            break  # 连接断开，触发重连
            except Exception:
                await asyncio.sleep(3)  # 连接失败等待3秒

    def _send_worker(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._async_send_loop())
        loop.close()

    def start(self):
        if self.send_thread is None or not self.send_thread.is_alive():
            self.stop_event.clear()
            self.send_thread = threading.Thread(target=self._send_worker, daemon=True)
            self.send_thread.start()

    def put_frame(self, image_bytes, data):
        if self.stop_event.is_set(): return
        if self.send_queue.full():
            try:
                self.send_queue.get_nowait()  # 丢弃旧帧
            except queue.Empty:
                pass
        try:
            self.send_queue.put_nowait(FramePackage(image_bytes, data))
        except queue.Full:
            pass

# ==========================
# Modbus TCP 从站
# ==========================
class ModbusWriter:

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
        b = struct.pack('>f', float(value))
        if order == "BADC":
            b = b[1:2] + b[0:1] + b[3:4] + b[2:3]
        elif order == "CDAB":
            b = b[2:4] + b[0:2]
        return list(struct.unpack('>HH', b))

    @staticmethod
    def _init_context():
        store = ModbusSlaveContext(hr=ModbusSequentialDataBlock(0, [0]*120))
        return ModbusServerContext(slaves=store, single=True)

    def send_position_6axis(self, x, y, z, rs, ry, rz, float_order="BADC"):
        regs_start = [self.REG_X, self.REG_Y, self.REG_Z,
                      self.REG_SPEED, self.REG_RY, self.REG_RZ]
        values = [x, y, z, rs, ry, rz]
        regs = []
        for v in values:
            regs.extend(self.float_to_regs(v, order=float_order))
        # slave id 1 (与原代码一致)
        self.context[1].setValues(3, regs_start[0], regs)
        print(f"✅ 写入机械臂6轴数据：{[round(v,3) for v in values]}")

    def send_all_flag(self, flag9, flag10, flag11, flag12, float_order="BADC"):
        regs_start = [self.REG_9_FLAG, self.REG_10_FLAG,
                      self.REG_11_FLAG, self.REG_12_FLAG]
        values = [flag9, flag10, flag11, flag12]
        regs = []
        for v in values:
            regs.extend(self.float_to_regs(v, order=float_order))
        self.context[1].setValues(3, regs_start[0], regs)
        print(f"✅ 写入标志位数据：{[round(v,3) for v in values]}")

    def start_server(self):
        def run_server():
            print(f"🚀 启动 Modbus TCP 从站，等待工控机连接...")
            StartTcpServer(self.context, address=(self.ip, self.port))
        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
        print("✅ Modbus 服务已后台启动")

# ==========================
# 图像处理 + YOLO OBB + ByteTrack（优先） + 回退策略（保留 OBB）
# ==========================
class ImageHandler:
    roi_x_left = 510
    roi_x_right = 1040
    real_width_cm = 59.8
    real_height_cm = 74.0
    pixels_per_mmx = 59.8 / 515 * 10
    pixels_per_mmy = 50 / 435 * 10
    pixels_per_mm = 598 / 505
    pixels_per_cm = 59.8 / 505

    def __init__(self, model, modbus=None, conf_thresh=0.3, device="cuda"):       #降低检测阈值conf_thresh
        self.model = model
        self.modbus = modbus
        self.device = device
        self.conf_thresh = conf_thresh
        self.lock = threading.Lock()

        self.tracker_initialized = False  # 标记 ByteTrack 是否已初始化
        self.tracker_config_path = "bytetrack.yaml"  # ByteTrack 配置路径

        # 默认用 SORT（保留） — 我们会尝试用 ultralytics 的 ByteTrack 调用作为优先选项
        self.tracker_sort = Sort(max_age=10, min_hits=1, iou_threshold=0.3)


        # internal state (保留原有)
        self.dot_list = []             # 用于速度估计
        self.object_line = {"line0": False, "line1": False, "line2": False}
        self.targets = None
        self.SPEED_NOW = 0
        self.delay_time = 0
        self.cz_mm = 20
        self.LAST_FLAG1 = True
        self.dot_list_maxlen = 30
        self.trigger_line = {"bottom_y": None, "middle_y": None, "top_y": None}
        self.time_dig = [0.0, 0.0, 0.0]
        self.object_count = 0
        self.image = None
        self.h, self.w = None, None
        self.delay_time = 0

        self.objectcount = 0
        # 在 __init__ 中新增字典，记录每个 track_id 是否已经计数过
        self.counted_ids = set()

        self.result_data = None  # 也同步初始化到对象属性
        self.result_data = None
        self.last_send_time = 0  # 上次发送时间

        # [新增] 启动 WebSocket 发送器
        self.sender = VideoSender(Config())
        self.sender.start()

    def draw_lines(self, image):
        self.image = image

        self.h, self.w = image.shape[:2]
        # 基于图像计算的比例
        self.roi_w_px = ImageHandler.roi_x_right - ImageHandler.roi_x_left
        self.pixels_per_cm_x = self.roi_w_px / ImageHandler.real_width_cm
        self.pixels_per_cm_y = self.h / ImageHandler.real_height_cm
        # ROI（拷贝，避免原图被修改）
        self.roi = self.image[:, ImageHandler.roi_x_left:ImageHandler.roi_x_right].copy()
        self.trigger_line = {"bottom_y": 2*self.h//3, "middle_y": 2*self.h//5, "top_y": self.h//5}

        # 绘制触发线（保留可视化）
        # cv2.line(image, (0, self.trigger_line["bottom_y"]), (self.roi_x_right, self.trigger_line["bottom_y"]), (255,0,0), 2)
        # cv2.line(image, (0, self.trigger_line["top_y"]), (self.roi_x_right, self.trigger_line["top_y"]), (255,0,0), 2)
        return image

    def _run_track_with_model_track(self, roi_for_track):
        """
        尝试使用 ultralytics 的 model.track（ByteTrack）来获得跟踪信息。
        返回: results 或 None（失败）
        注意：roi_for_track 应该是 numpy RGB/BGR 图像（与 model.track 接收的格式一致）
        """
        # self.byte_tracker = BYTETracker(track_thresh=0.25, match_thresh=0.85, track_buffer=90, frame_rate=30)
        try:
            # model.track 内部会做 detect + track（ByteTrack），我们希望获得 boxes + ids
            # 使用 persist=True 保持 ID 连续，tracker 可留空或使用 bytetrack.yaml
            results = self.model.track(
                source=roi_for_track,
                tracker="bytetrack.yaml",# 若你没有此 yaml，可改为 None（使用默认）
                persist=True,
                stream=True,
                conf=self.conf_thresh,
                device=self.device,
                verbose=False
            )
            return results
        except Exception as e:
            # 跟踪失败时打印，并回退到单帧检测
            print(f"[WARN] model.track 调用失败，回退到单帧检测：{e}")
            return None

    # ========================== 完整修改版 ==========================
    # ✅ 新增逻辑：当同一帧检测到多个目标时，仅处理 ID 最小的那个目标
    # ===============================================================
    def detection_logic(self, depth_img=None):
        """
        主检测逻辑（修正版）：
         - 仍优先尝试 model.track（ByteTrack），但：即使 track 成功也会跑一次 OBB，用于角度/位置/触发逻辑一致
         - 严格只让“主目标/激活目标”进入触发线/速度/Modbus 状态机，避免多目标污染全局变量
         - delay_time 防负数，避免瞬间触发/复位导致机械臂不下抓
        """
        # [新增] 预先定义变量，防止未检测到物体时发送数据报错
        cx_mm, cy_mm, angle_deg, conf = 0.0, 0.0, 0.0, 0.0

        result_data = None



        if depth_img is not None:
            self.depth_image = depth_img

        # ✅ 新增：激活目标ID（绑定同一个物体走完整状态机）
        if not hasattr(self, "active_track_id"):
            self.active_track_id = None

        with self.lock:
            now = time.time()


            # ROI 图像安全检查
            if not hasattr(self, "image") or self.image is None:
                print("[WARN] image 未初始化，跳过检测")
                return None

            roi_img = self.image[:, self.roi_x_left:self.roi_x_right]
            if roi_img is None or roi_img.size == 0:
                print("[WARN] ROI 图像为空，跳过检测")
                return self.image, {}

            # ------------------------------
            # 1) 尝试 ByteTrack（只拿 ID 和 xyxy）
            # ------------------------------
            results_track = None
            track_ok = False
            track_boxes_xyxy = None
            track_ids = None


            try:
                results_track = self._run_track_with_model_track(roi_img)
                if results_track is not None and not isinstance(results_track, list):
                    results_track = list(results_track)
            except Exception as e:
                print("[WARN] 调用 model.track 异常:", e)
                results_track = None

            if results_track is not None and len(results_track) > 0:
                r0 = results_track[0]
                if hasattr(r0, "boxes") and r0.boxes is not None and len(r0.boxes) > 0:
                    try:
                        if hasattr(r0.boxes, "xyxy") and r0.boxes.xyxy is not None:
                            track_boxes_xyxy = r0.boxes.xyxy.cpu().numpy()
                        elif hasattr(r0.boxes, "xywh") and r0.boxes.xywh is not None:
                            xywh = r0.boxes.xywh.cpu().numpy()
                            track_boxes_xyxy = np.zeros((xywh.shape[0], 4))
                            track_boxes_xyxy[:, 0] = xywh[:, 0] - xywh[:, 2] / 2
                            track_boxes_xyxy[:, 1] = xywh[:, 1] - xywh[:, 3] / 2
                            track_boxes_xyxy[:, 2] = xywh[:, 0] + xywh[:, 2] / 2
                            track_boxes_xyxy[:, 3] = xywh[:, 1] + xywh[:, 3] / 2
                        else:
                            track_boxes_xyxy = None

                        if track_boxes_xyxy is not None and len(track_boxes_xyxy) > 0:

                            if r0.boxes.id is not None:
                                track_ids = r0.boxes.id.cpu().numpy().astype(int)
                            else:
                                track_ids = np.arange(len(track_boxes_xyxy)).astype(int)
                            track_ok = True
                    except Exception as e:
                        print("[WARN] 解析 track 输出失败:", e)
                        track_ok = False

            # ------------------------------
            # 2) 统一跑一遍单帧 OBB（关键修复：保证触发/角度/位置逻辑始终可用）
            # ------------------------------
            try:
                with torch.no_grad():
                    results = self.model(roi_img, device=self.device, conf=self.conf_thresh, verbose=False)
            except Exception as e:
                print("[ERROR] 单帧推理失败：", e)
                return self.image, {}

            r = results[0]
            if not (hasattr(r, "obb") and r.obb is not None and hasattr(r.obb, "xywhr") and len(r.obb.xywhr) > 0):
                # 没有 OBB：此时即使 track_ok，也缺角度等信息，保守返回
                return self.image, {}

            obb_boxes = r.obb.xywhr.cpu().numpy()
            classes = r.obb.cls.cpu().numpy().astype(int) if r.obb.cls is not None else np.zeros(len(obb_boxes),
                                                                                                 dtype=int)
            confs = r.obb.conf.cpu().numpy() if r.obb.conf is not None else np.ones(len(obb_boxes))

            # ------------------------------
            # 3) 生成 tracks：优先 ByteTrack 的 ID；否则用你原来的 SORT
            # ------------------------------
            tracks_all = []

            if track_ok:
                # 用 ByteTrack 的 ID + xyxy 直接构造 tracks（不再让多目标跑 SORT 污染ID）
                for (x1, y1, x2, y2), tid in zip(track_boxes_xyxy, track_ids):
                    tracks_all.append([float(x1), float(y1), float(x2), float(y2), int(tid)])
            else:
                # 回到你原来的 SORT：用 OBB 转 xyxy + conf
                detections = []
                for (cx, cy, w, h, ang), conf in zip(obb_boxes, confs):
                    x1 = cx - w / 2
                    y1 = cy - h / 2
                    x2 = cx + w / 2
                    y2 = cy + h / 2
                    detections.append([x1, y1, x2, y2, float(conf)])
                detections_np = np.array(detections) if len(detections) > 0 else []
                tracks_all = self.tracker_sort.update(detections_np) if len(detections_np) > 0 else []

            if tracks_all is None or len(tracks_all) == 0:
                return self.image, {}

            # tracks_all0 = []
            # if len(track_boxes_xyxy) > 0:
            #     for box, tid in zip(track_boxes_xyxy, track_ids):
            #         tracks_all0.append([*box, tid])

            # 如果还是空的，直接发送空数据并返回
            # if len(tracks_all0) == 0:
            #     self._send_to_websocket_image(self.image)  # 发送空画面
            #     self._send_to_websocket(self.image, 0, 0, 0, 0, 0, 0)  # 发送空画面
            #     return self.image, {}

            # ------------------------------
            # 4) ✅ 选择“当前要驱动状态机”的唯一目标：active_track_id 优先，其次最小ID
            # ------------------------------
            try:
                tracks_all = sorted(tracks_all, key=lambda x: int(x[4]))
            except Exception:
                pass


            if self.active_track_id is not None:
                picked = [t for t in tracks_all if int(t[4]) == int(self.active_track_id)]
                if len(picked) > 0:
                    tracks = picked
                else:
                    # 激活ID丢了：退回主目标继续跑（避免卡死）
                    tracks = [tracks_all[0]]
                    self.active_track_id = int(tracks[0][4])
            else:
                tracks = [tracks_all[0]]

            main_track_id = int(tracks[0][4])

            # ------------------------------
            # 5) 绘制：可以画所有目标，但后续业务逻辑只跑 tracks（唯一目标）
            # ------------------------------
            try:
                for track in tracks_all:
                    x1, y1, x2, y2, track_id = track
                    # 找最接近的 OBB
                    i = int(np.argmin(
                        [np.hypot((cx - (x1 + x2) / 2), (cy - (y1 + y2) / 2)) for (cx, cy, _, _, _) in obb_boxes]
                    ))
                    cx, cy, w, h, ang = obb_boxes[i]
                    cls_id = int(classes[i]) if len(classes) > i else 0
                    conf = float(confs[i]) if len(confs) > i else 1.0

                    abs_cx = int(cx + self.roi_x_left)
                    abs_cy = int(cy)

                    angle_deg = math.degrees(ang)
                    rect = ((abs_cx, abs_cy), (int(w), int(h)), angle_deg)
                    box_pts = cv2.boxPoints(rect)
                    box_pts = np.intp(box_pts)
                    cv2.drawContours(self.image, [box_pts], 0, (0, 0, 255), 2)
                    name = self.model.names[cls_id] if hasattr(self.model, "names") else f"class{cls_id}"
                    cv2.putText(self.image, f"ID {int(track_id)} {name} {conf:.2f}",
                                (abs_cx + 5, abs_cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            except Exception as e:
                print("[WARN] 绘制阶段异常:", e)

            # ------------------------------
            # 6) ✅ 只对唯一目标执行你原来的完整业务逻辑
            # ------------------------------
            for track in tracks:
                x1, y1, x2, y2, track_id = track
                track_id = int(track_id)

                # 找最接近的 OBB（用于角度/宽高/中心）
                i = int(np.argmin(
                    [np.hypot((cx - (x1 + x2) / 2), (cy - (y1 + y2) / 2)) for (cx, cy, _, _, _) in obb_boxes]
                ))
                cx, cy, w, h, ang = obb_boxes[i]
                cls_id = int(classes[i]) if len(classes) > i else 0
                conf = float(confs[i]) if len(confs) > i else 1.0

                abs_cx = int(cx + self.roi_x_left)
                abs_cy = int(cy)

                # 角度处理（保留你原逻辑）
                angle_deg = math.degrees(ang)
                if h > w:
                    angle_deg += 90.0
                angle_deg = (angle_deg + 180) % 360 - 180
                if angle_deg > 90:
                    angle_deg -= 180
                elif angle_deg < -90:
                    angle_deg += 180
                angle_deg = -angle_deg

                # 深度融合（保留你原逻辑）
                if depth_img is not None and 0 <= abs_cy < depth_img.shape[0] and 0 <= abs_cx < depth_img.shape[1]:
                    depth_val = float(depth_img[abs_cy, abs_cx])
                    self.cz_mm = max(0, 1190.0 - depth_val)

                # 绘制（保留你原逻辑）
                cv2.circle(self.image, (abs_cx, abs_cy), 6, (0, 0, 255), 2)
                name = self.model.names[cls_id] if hasattr(self.model, "names") else f"class{cls_id}"
                cv2.putText(self.image, f"ID {int(track_id)} {name} {conf:.2f}",
                            (abs_cx + 5, abs_cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                rect = ((abs_cx, abs_cy), (int(w), int(h)), math.degrees(ang))
                box_pts = cv2.boxPoints(rect)
                box_pts = np.intp(box_pts)
                cv2.drawContours(self.image, [box_pts], 0, (0, 0, 255), 2)

                # 像素->mm & dot_list & 触发线 & modbus 保留原逻辑
                cx_mm = -(abs_cy * ImageHandler.pixels_per_mm)
                cy_mm = (self.roi_x_right - abs_cx) * ImageHandler.pixels_per_mm

                # ✅ dot_list 只对唯一目标记录（现在 tracks 只有一个）
                self.dot_list.append((abs_cy, now))
                if len(self.dot_list) > self.dot_list_maxlen:
                    self.dot_list.pop(0)

                # 初始化 targets（保留你原结构）
                if self.targets is None:
                    self.targets = {
                        "last_line1_time": None,
                        "last_line2_time": None,
                        "last_line1_flag": None,
                        "last_line2_flag": None,
                        "frame_count": 0,
                        "trajectory": 0,
                    }

                frame_objs = []

                # ✅ frame_count 只加一次（现在只跑一个目标）
                self.targets["frame_count"] += 1

                if self.targets["frame_count"] == 6:
                    self.object_count += 1
                    if depth_img is not None:
                        self.cz_mm = depth_img[abs_cy, abs_cx]
                        self.cz_mm = 1190 - self.cz_mm

                label = f"({cx_mm:.1f}mm, {cy_mm:.1f}mm, {self.cz_mm:.1f}mm)"
                cv2.putText(self.image, label, (abs_cx + 5, abs_cy + 40),
                            cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1)

                # === 触发线判断与后处理（保留你的逻辑） ===
                if abs_cy > self.trigger_line["bottom_y"]:
                    self.object_line["line0"] = True
                    self.time_dig[0] = time.time()

                    # ✅ 绑定激活目标（关键：保证后续line1/line2只跟同一物体）
                    if self.active_track_id is None:
                        self.active_track_id = int(track_id)

                    if track_id not in self.counted_ids:
                        self.objectcount += 1
                        self.counted_ids.add(track_id)
                        print("计数器 +1，现在 objectcount =", self.objectcount)

                    if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                        try:
                            self.modbus.send_all_flag(0, 0, 0, 0)
                        except Exception as e:
                            print("[WARN] send_all_flag failed:", e)
                    print("检测到物体")

                if self.trigger_line["top_y"] <= abs_cy <= self.trigger_line["bottom_y"] and self.object_line["line0"]:
                    self.object_line["line1"] = True
                    self.time_dig[1] = time.time()
                    if self.targets["last_line1_flag"] is None:
                        self.targets["last_line1_time"] = time.time()
                        self.targets["last_line1_flag"] = True

                    if self.cz_mm > 300 or self.cz_mm < 0:
                        self.cz_mm = 40

                    if self.modbus is not None and hasattr(self.modbus, "send_position_6axis"):
                        try:
                            self.modbus.send_position_6axis(cx_mm, cy_mm, self.cz_mm / 2, 0, 0, -angle_deg)
                        except Exception as e:
                            print("[WARN] send_position_6axis failed:", e)

                    if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                        try:
                            self.modbus.send_all_flag(1, 0, 0, 0)
                        except Exception as e:
                            print("[WARN] send_all_flag failed:", e)

                    try:
                        lines = [f"{cx_mm, cy_mm, self.cz_mm / 2, 0, 0, -angle_deg}\n"]
                        with open(r"D:\examples_project\python projects\location.txt", "a", encoding="utf-8") as f:
                            f.writelines(lines)
                    except Exception as e:
                        print("[WARN] 写入 location.txt 失败:", e)

                if abs_cy < self.trigger_line["top_y"] and self.object_line["line1"]:
                    self.object_line["line2"] = True
                    self.counted_ids.clear()
                    self.time_dig[2] = time.time()
                    if self.targets["last_line2_flag"] is None:
                        self.targets["last_line2_flag"] = True

                        if len(self.dot_list) >= 4:
                            cy_vals = np.array([item[0] for item in self.dot_list[-10:]])
                            t_vals = np.array([item[1] for item in self.dot_list[-10:]])
                            t_vals = t_vals - t_vals[0]

                            if np.ptp(t_vals) > 1e-3:
                                A = np.vstack([t_vals, np.ones_like(t_vals)]).T
                                a, b = np.linalg.lstsq(A, cy_vals, rcond=None)[0]

                                speed_pix_per_s = abs(a)
                                self.SPEED_NOW = (speed_pix_per_s * ImageHandler.pixels_per_mm)

                                if not hasattr(self, "_speed_hist"):
                                    self._speed_hist = []
                                self._speed_hist.append(self.SPEED_NOW)
                                if len(self._speed_hist) > 5:
                                    self._speed_hist.pop(0)
                                self.SPEED_NOW = float(np.mean(self._speed_hist))
                                print("平滑速度 mm/s:", round(self.SPEED_NOW, 2))

                        if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                            try:
                                self.modbus.send_all_flag(1, 1, 0, 1)
                            except Exception as e:
                                print("[WARN] send_all_flag failed:", e)

                        self.targets["last_line2_time"] = time.time()
                        print(self.targets["last_line2_time"])

                        if self.modbus is not None and hasattr(self.modbus, "send_position_6axis"):
                            try:
                                self.modbus.send_position_6axis(0, 0, 0, 0, round(self.SPEED_NOW, 2), -angle_deg)
                            except Exception as e:
                                print("[WARN] send_position_6axis failed:", e)

                        try:
                            self.delay_time = 340 / self.SPEED_NOW if self.SPEED_NOW != 0 else 0
                        except Exception as e:
                            self.delay_time = 0
                            print(e)

                        # ✅ delay_time 防负数/异常
                        self.delay_time = max(0.0, float(self.delay_time))
                        print("速度mm/s, 触发等待时间", round(self.SPEED_NOW, 2), self.delay_time)

            # ===== 后续触发与跟随逻辑（保留你的原有分支逻辑） =====
            if self.object_line["line2"]:

                if self.SPEED_NOW < 100:
                    print(1)
                    self.delay_time = self.delay_time
                    self.delay_time -= 0.005
                elif 100 <= self.SPEED_NOW < 200:
                    self.delay_time -= 0.012
                elif 200 <= self.SPEED_NOW < 250:
                    self.delay_time -= 0.02
                elif 250 <= self.SPEED_NOW < 300:
                    self.delay_time -= 0.03
                elif 300 <= self.SPEED_NOW < 350:
                    self.delay_time -= 0.035
                elif 350 <= self.SPEED_NOW < 400:
                    self.delay_time -= 0.043
                elif 400 <= self.SPEED_NOW < 450:
                    self.delay_time -= 0.065
                elif 450 <= self.SPEED_NOW < 550:
                    self.delay_time -= 0.095
                elif 550 <= self.SPEED_NOW < 600:
                    self.delay_time -= 0.12
                elif 600 <= self.SPEED_NOW < 650:
                    self.delay_time -= 0.15
                elif 650 <= self.SPEED_NOW < 700:
                    self.delay_time -= 0.15
                elif self.SPEED_NOW >= 700:
                    self.delay_time -= 1

                # ✅ delay_time 防负数（关键）
                self.delay_time = max(0.0, float(self.delay_time))
                print(self.delay_time)

                if self.targets and self.targets.get("last_line2_time") is not None:
                    print("123")
                    print(self.targets["last_line2_time"])
                    print(time.time())
                    if time.time() - self.targets["last_line2_time"] > self.delay_time:
                        for i in range(1, 20):
                            print(f"开始跟随,物体{self.object_count}")
                        if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                            print("aaaaaaaaaaaaaaaaa")
                            try:
                                self.modbus.send_all_flag(0, 0, 1, 0, float_order="BADC")
                            except Exception as e:
                                print("[WARN] send_all_flag failed:", e)

                        # 重置对象状态
                        for key in self.object_line:
                            self.object_line[key] = False
                        self.targets = None
                        self.LAST_FLAG1 = True
                        self.cz_mm = 20
                        self.delay_time = 0
                        self.SPEED_NOW = 0

                        # ✅ 清掉激活ID（关键）
                        self.active_track_id = None

            if self.object_line["line1"] and not self.object_line["line2"]:
                if time.time() - self.time_dig[1] > 10:
                    for key in self.object_line:
                        self.object_line[key] = False
                    self.targets = None
                    self.LAST_FLAG1 = True
                    self.cz_mm = 20
                    self.delay_time = 0
                    self.SPEED_NOW = 0
                    self.dot_list.clear()

                    # ✅ 清掉激活ID（关键）
                    self.active_track_id = None

                    if self.modbus is not None and hasattr(self.modbus, "send_all_flag"):
                        try:
                            self.modbus.send_all_flag(0, 0, 0, 0, float_order="BADC")
                            self.modbus.send_position_6axis(0, 0, 0, 0, 0, 0, float_order="BADC")
                        except Exception as e:
                            print("[WARN] send_all_flag/send_position_6axis failed:", e)

                    for i in range(10):
                        print("出现异常，进行清零操作")

            self._send_to_websocket_image(self.image)  # 发送空画面

            # --- [新增] 发送数据到 WebSocket ---
            self._send_to_websocket_false(self.image)
            # self._send_to_websocket(self.image, cx_mm, cy_mm, self.cz_mm, angle_deg, self.objectcount,
            #                               self.SPEED_NOW, conf)
            cv2.imshow('123', self.image)

            return self.image, (result_data if result_data is not None else {})

    def _send_to_websocket_false(self, img):
        """辅助函数：打包并发送"""
        success, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if success:
            data = {
                "location_x": random.uniform(100, 200),
                "location_y": random.uniform(90, 110),
                "location_z": random.uniform(30, 40),
                "location_rz": random.uniform(40, 60),
                "speed": random.uniform(0.4, 0.5),
                "confidence": random.uniform(0.8, 1),
                "count": 10,
                # 建议加上时间戳，方便前端计算延迟
                "timestamp": time.time()
            }
            self.sender.put_frame(buffer.tobytes(), data)

    def _send_to_websocket_image(self, img):
        """辅助函数：打包并发送"""
        success, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if success:
            data = {
                "location_x": 0,
                "location_y": 0,
                "location_z": 0,
                "location_rz": 0,
                "speed": 0,
                "confidence": 0,
                "count": 0,
                # 建议加上时间戳，方便前端计算延迟
                "timestamp": time.time()
            }
            self.sender.put_frame(buffer.tobytes(), data)

# ==========================
# 相机处理
# ==========================
class CameraHandler:
    def __init__(self, device_index=0):
        self.device_index = device_index
        self.camera = None
        self.stop_flag = False
        # 这里暂时填 None，下面主程序会在实例化 Handler 时传入正确的 Modbus 对象
        # 但为了兼容你原来直接用全局 Modbus 的写法，我们会在创建 Handler 之前创建 Modbus
        self.Handler = None

        # stable ID 管理（在 __init__ 中）
        self.next_stable_id = 1
        self.tracker_to_stable = {}  # tracker_id -> (stable_id, last_seen_ts)
        self.stable_timeout = 1.5  # 秒，丢失后多少秒回收 id（按需调整）

    def _map_tracker_to_stable(self, tracker_id):
        """将底层 tracker id 映射到稳定 stable_id，并更新 last_seen 时间。"""
        now = time.time()
        tracker_id = int(tracker_id)
        if tracker_id in self.tracker_to_stable:
            stable, _ = self.tracker_to_stable[tracker_id]
            self.tracker_to_stable[tracker_id] = (stable, now)
            return stable
        stable = self.next_stable_id
        self.next_stable_id += 1
        self.tracker_to_stable[tracker_id] = (stable, now)
        return stable

    def _cleanup_stable_map(self):
        """清理超时未见的 tracker->stable 映射，防止字典无限增长。"""
        now = time.time()
        to_delete = [tid for tid, (_, last) in self.tracker_to_stable.items() if now - last > self.stable_timeout]
        for tid in to_delete:
            del self.tracker_to_stable[tid]

    def open_camera(self):
        nDeviceNum = ctypes.c_uint(0)
        ret = Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet|DeviceType_USB|DeviceType_Ethernet_Vir|DeviceType_USB_Vir, byref(nDeviceNum))
        if ret !=0 or nDeviceNum.value==0:
            raise RuntimeError("未检测到相机")
        stDeviceList = MV3D_RGBD_DEVICE_INFO_LIST()
        Mv3dRgbd.MV3D_RGBD_GetDeviceList(DeviceType_Ethernet|DeviceType_USB|DeviceType_Ethernet_Vir|DeviceType_USB_Vir, pointer(stDeviceList.DeviceInfo[0]), 20, byref(nDeviceNum))
        self.camera = Mv3dRgbd()
        ret = self.camera.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[self.device_index]))
        if ret!=0:
            raise RuntimeError("打开相机失败")
        ret = self.camera.MV3D_RGBD_Start()
        if ret!=0:
            self.camera.MV3D_RGBD_CloseDevice()
            raise RuntimeError("启动取流失败")
        print("[INFO] 相机已启动")

    def stream_frames(self):
        """循环获取并显示彩色图和深度图"""
        if self.camera is None:
            raise RuntimeError("相机未打开")

        print("[INFO] 开始实时取流，按 'q' 退出")
        while not self.stop_flag:
            stFrameData = MV3D_RGBD_FRAME_DATA()
            ret = self.camera.MV3D_RGBD_FetchFrame(pointer(stFrameData),5000)
            if ret != MV3D_RGBD_OK:
                continue
            color_img, depth_img = None, None
            for i in range(stFrameData.nImageCount):
                img_type = stFrameData.stImageData[i].enImageType
                width = stFrameData.stImageData[i].nWidth
                height = stFrameData.stImageData[i].nHeight
                data_len = stFrameData.stImageData[i].nDataLen
                img_data = string_at(stFrameData.stImageData[i].pData,data_len)
                if img_type==ImageType_Depth:
                    depth_values = struct.unpack('H'*(len(img_data)//2),img_data)
                    depth_img = np.array(depth_values,dtype=np.uint16).reshape(height,width)
                elif img_type==ImageType_RGB8_Planar:
                    plane_size = width*height
                    img_array = np.frombuffer(img_data,dtype=np.uint8)
                    if len(img_array)>=plane_size*3:
                        r = img_array[0:plane_size].reshape(height,width)
                        g = img_array[plane_size:2*plane_size].reshape(height,width)
                        b = img_array[2*plane_size:3*plane_size].reshape(height,width)
                        color_img = cv2.merge([b,g,r])
                        # 把当前帧传给 Handler
                        if self.Handler is not None:
                            self.Handler.draw_lines(color_img)
                            # detection_logic 可能返回 None（若无 image），但我们不依赖返回值
                            self.Handler.detection_logic(depth_img)
            if color_img is not None:
                cv2.imshow("RGB Image",color_img)
            if depth_img is not None:
                depth_norm = cv2.normalize(depth_img,None,0,255,cv2.NORM_MINMAX)
                depth_colormap = cv2.applyColorMap(depth_norm.astype(np.uint8),cv2.COLORMAP_JET)
                cv2.imshow("Depth Image",depth_colormap)
            if cv2.waitKey(1)&0xFF==ord('q'):
                self.stop_flag=True
                break
        cv2.destroyAllWindows()

    def start_streaming(self):
        self.stop_flag = False
        t = threading.Thread(target=self.stream_frames,daemon=True)
        t.start()

    def close_camera(self):
        self.stop_flag=True
        time.sleep(0.5)
        if self.camera:
            self.camera.MV3D_RGBD_Stop()
            self.camera.MV3D_RGBD_CloseDevice()
            self.camera=None
            print("[INFO] 相机已关闭")

# ==========================
# 主程序
# ==========================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    # 创建 Modbus 并启动（保证在 CameraHandler/ ImageHandler 使用之前已创建）
    Modbus = ModbusWriter(ip="0.0.0.0", port=502)
    Modbus.start_server()

    # 创建相机 handler 并把 Modbus 传入 ImageHandler
    cam = CameraHandler()
    # 在 CameraHandler 中创建 Handler，保证 Modbus 已定义
    cam.Handler = ImageHandler(model=Model, modbus=Modbus, conf_thresh=0.3, device="cuda")

    cam.open_camera()
    cam.start_streaming()

    # 主线程等待，直到用户按 q 退出
    while not cam.stop_flag:
        time.sleep(0.1)

    cam.close_camera()
