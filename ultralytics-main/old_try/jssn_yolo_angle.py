import os
import time
import math
import struct
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from ultralytics import YOLO
import cv2
from pymodbus.client import ModbusTcpClient
import numpy as np

# ----------------- 配置 -----------------
WATCH_DIR = r'./camera/photo'
SAVE_DIR = r'./camera/detect'
MODEL_PATH = r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\project\pillow4\weights\best.pt'

os.makedirs(WATCH_DIR, exist_ok=True)
os.makedirs(SAVE_DIR, exist_ok=True)

PHYS_W = 1750
PHYS_H = 1250
Z_HEIGHT = 210.0

# 机械臂 Modbus 配置
#ROBOT_IP = "192.168.0.30"
ROBOT_IP = "169.254.0.66"
ROBOT_PORT = 502
REG_X = 0
REG_Y = 2
REG_Z = 4
REG_A = 6
REG_zw = 8
FLOAT_ORDER = "BADC"

# ----------------- Modbus TCP -----------------
def float_to_regs(value, order="BADC"):
    b = struct.pack('>f', value)
    byte_map = {
        "ABCD": [0, 1, 2, 3],
        "DCBA": [3, 2, 1, 0],
        "BADC": [1, 0, 3, 2],
        "CDAB": [2, 3, 0, 1]
    }
    b_ordered = bytes([b[i] for i in byte_map[order]])
    return struct.unpack('>HH', b_ordered)


def send_position(client, x_mm, y_mm, z_mm, angle_deg, float_order=FLOAT_ORDER, pulse_zw=True):
    """发送机械臂坐标，同时自动产生动作信号短脉冲"""
    # 坐标寄存器
    x_regs = float_to_regs(x_mm, order=float_order)
    y_regs = float_to_regs(y_mm, order=float_order)
    z_regs = float_to_regs(z_mm, order=float_order)
    a_regs = float_to_regs(angle_deg, order=float_order)
    zw_regs_zero = float_to_regs(0.0, order=float_order)

    # 先写坐标 + zw = 0
    client.write_registers(REG_X, list(x_regs))
    client.write_registers(REG_Y, list(y_regs))
    client.write_registers(REG_Z, list(z_regs))
    client.write_registers(REG_A, list(a_regs))
    client.write_registers(REG_zw, list(zw_regs_zero))

    if pulse_zw:
        # 短脉冲置1
        zw_regs_one = float_to_regs(1.0, order=float_order)
        client.write_registers(REG_zw, list(zw_regs_one))
        print(f"✅ 坐标已发送，REG_zw置1: X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f}, A={angle_deg:.1f}")
        time.sleep(0.1)  # 短脉冲 100ms
        # 回到0
        client.write_registers(REG_zw, list(zw_regs_zero))
        print("🔹 REG_zw已恢复为0")
    else:
        print(f"✅ 坐标已发送: X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f}, A={angle_deg:.1f}, REG_zw=0")

# ----------------- 加载模型 -----------------
model = YOLO(MODEL_PATH)

# ----------------- 事件处理 -----------------
class NewImageHandler(FileSystemEventHandler):
    global Z_HEIGHT
    def __init__(self, modbus_client):
        super().__init__()
        self.client = modbus_client

    def on_created(self, event):
        if not event.is_directory and event.src_path.lower().endswith(('.jpg', '.png', '.jpeg')):
            time.sleep(0.5)
            print(f"\n🖼 检测新图片: {event.src_path}")
            self.run_yolo(event.src_path)

    def run_yolo(self, img_path):
        results = model.predict(source=img_path, task="obb", save=False)
        img_h, img_w = results[0].orig_shape[:2]
        img_plot = results[0].orig_img.copy()

        # 绘制自定义坐标轴
        origin = (img_w - 1, 0)
        cv2.line(img_plot, origin, (img_w - 1, img_h - 1), (0, 0, 255), 2)  # X轴
        cv2.line(img_plot, origin, (0, 0), (255, 0, 0), 2)  # Y轴
        cv2.putText(img_plot, 'X', (img_w - 15, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img_plot, 'Y', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        any_detected = False

        for r in results:
            if r.obb is not None and len(r.obb.xywhr) > 0:
                any_detected = True
                for idx, box in enumerate(r.obb.xywhr.cpu().numpy(), start=1):
                    cx, cy, w, h, angle = box
                    phys_x = (0 - cy / img_h) * PHYS_H
                    phys_y = ((img_w - cx) / img_w) * PHYS_W - 710

                    pts = cv2.boxPoints(((cx, cy), (w, h), math.degrees(angle)))
                    pts = pts.astype(int)
                    cv2.drawContours(img_plot, [pts], 0, (0, 255, 0), 2)
                    cv2.circle(img_plot, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    cv2.putText(img_plot, f"#{idx}", (int(cx) + 5, int(cy) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # YOLO水平线
                    cv2.line(img_plot, (0, int(cy)), (img_w - 1, int(cy)), (255, 255, 0), 1, lineType=cv2.LINE_AA)
                    cv2.putText(img_plot, 'YOLO Horizontal', (10, int(cy)-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                    # 计算长边角度
                    edges = [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
                    edge_lengths = [np.linalg.norm(np.array(p2) - np.array(p1)) for p1, p2 in edges]
                    longest_idx = np.argmax(edge_lengths)
                    p1, p2 = edges[longest_idx]
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    final_angle = math.degrees(math.atan2(dy, dx))



                    if final_angle < 0:
                        final_angle = 180 + final_angle




                    cv2.line(img_plot, tuple(p1), tuple(p2), (0, 255, 255), 2)
                    cv2.putText(img_plot, f"Angle: {final_angle:.1f}", (int(cx), int(cy)-15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    # 发送机械臂坐标，动作信号短脉冲
                    send_position(self.client, phys_x, phys_y, Z_HEIGHT, final_angle, pulse_zw=True)
                    print(f"⏱ 检测框 {idx} 坐标已发送，等待 5 分钟再发送下一个目标...")
                    time.sleep(20)  # 测试用10秒，正式改300秒

        # 若没有目标，也发送0信号保持REG_zw为0
        if not any_detected:
            send_position(self.client, 0, 0, 0, 0, pulse_zw=False)

        # 保存结果
        filename = os.path.basename(img_path)
        save_path = os.path.join(SAVE_DIR, f"detect_{filename}")
        cv2.imwrite(save_path, img_plot)
        print(f"✅ 检测结果已保存: {save_path}")


# ----------------- 主程序 -----------------
if __name__ == "__main__":
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT)
    if not client.connect():
        print("❌ 连接机械臂失败")
        exit()
    print("✅ 成功连接机械臂")

    event_handler = NewImageHandler(client)
    observer = Observer()
    observer.schedule(event_handler, WATCH_DIR, recursive=False)
    observer.start()
    print(f"📂 监听目录: {WATCH_DIR}，有新图片将自动检测并发送机械臂坐标...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
    client.close()
