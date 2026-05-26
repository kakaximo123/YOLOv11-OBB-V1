import os
import time
import math
import struct
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from ultralytics import YOLO
import cv2
from pymodbus.client import ModbusTcpClient

# ----------------- 配置 -----------------
WATCH_DIR = r'./camera/photo'  # 监听截图保存目录
SAVE_DIR = r'./camera/detect'  # 检测结果保存目录
MODEL_PATH = r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\runs\obb\train10\weights\best.pt'

os.makedirs(WATCH_DIR, exist_ok=True)
os.makedirs(SAVE_DIR, exist_ok=True)

PHYS_W = 1750  # 宽边对应 y 坐标
PHYS_H = 1250  # 高边对应 x 坐标
Z_HEIGHT = 200  # 固定Z高度 mm

# 机械臂 Modbus 配置
ROBOT_IP = "192.168.0.30"
ROBOT_PORT = 502
REG_X = 0
REG_Y = 2
REG_Z = 4
REG_A = 6
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

def send_position(client, x_mm, y_mm, z_mm, angle_deg, float_order=FLOAT_ORDER):
    x_regs = float_to_regs(x_mm, order=float_order)
    y_regs = float_to_regs(y_mm, order=float_order)
    z_regs = float_to_regs(z_mm, order=float_order)
    a_regs = float_to_regs(angle_deg, order=float_order)
    client.write_registers(REG_X, list(x_regs))
    client.write_registers(REG_Y, list(y_regs))
    client.write_registers(REG_Z, list(z_regs))
    client.write_registers(REG_A, list(a_regs))
    print(f"✅ 已发送: X={x_mm:.1f}mm, Y={y_mm:.1f}mm, Z={z_mm:.1f}mm, A={angle_deg:.1f}°")

# ----------------- 加载模型 -----------------
model = YOLO(MODEL_PATH)

# ----------------- 事件处理 -----------------
class NewImageHandler(FileSystemEventHandler):
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

        # 绘制坐标轴（右上角为原点）
        origin = (img_w - 1, 0)
        cv2.line(img_plot, origin, (img_w - 1, img_h - 1), (0, 0, 255), 2)  # X轴
        cv2.line(img_plot, origin, (0, 0), (255, 0, 0), 2)  # Y轴
        cv2.putText(img_plot, 'X', (img_w - 15, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img_plot, 'Y', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # 检测目标
        for r in results:
            if r.obb is not None and len(r.obb.xywhr) > 0:
                for idx, box in enumerate(r.obb.xywhr.cpu().numpy(), start=1):
                    cx, cy, w, h, angle = box

                    # 像素坐标 -> 实际物理坐标（右上角为原点）
                    phys_x = (0 - cy / img_h) * PHYS_H  # X纵向递减
                    phys_y = ((img_w - cx) / img_w) * PHYS_W  # Y横向增大

                    print(f"检测框 {idx}: 像素坐标 cx={cx:.1f}, cy={cy:.1f}")
                    print(
                        f"检测框 {idx}: 物理坐标 X={phys_x:.1f}mm, Y={phys_y:.1f}mm, Z={Z_HEIGHT}mm, angle={math.degrees(angle):.1f}°")

                    # 绘制检测框
                    pts = cv2.boxPoints(((cx, cy), (w, h), math.degrees(angle)))
                    pts = pts.astype(int)
                    cv2.drawContours(img_plot, [pts], 0, (0, 255, 0), 2)

                    # 绘制质心
                    cv2.circle(img_plot, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    # 绘制编号
                    cv2.putText(img_plot, f"#{idx}", (int(cx) + 5, int(cy) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                    # 发送机械臂
                    send_position(self.client, phys_x, phys_y, Z_HEIGHT, math.degrees(angle), FLOAT_ORDER)

                    # 每发送一个目标，等待 5 分钟
                    print(f"⏱ 检测框 {idx} 坐标已发送，等待 5 分钟再发送下一个目标...")
                    time.sleep(10)

            else:
                print("No OBB detected")

        # 保存带检测框和质心的图片
        filename = os.path.basename(img_path)
        save_path = os.path.join(SAVE_DIR, f"detect_{filename}")
        cv2.imwrite(save_path, img_plot)
        print(f"✅ 检测结果已保存: {save_path}")

# ----------------- 主程序 -----------------
if __name__ == "__main__":
    # 连接机械臂
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
