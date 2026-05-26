import os
import time
import math
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from ultralytics import YOLO
import cv2

# ----------------- 配置 -----------------
WATCH_DIR = r'./camera/photo'  # 监听截图保存目录
RESULT_DIR = r'./results'      # 检测结果保存目录
MODEL_PATH = r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\runs\obb\train10\weights\best.pt'

# 图片物理尺寸（单位 mm）
PHYS_W = 1750  # 宽边对应 y 坐标
PHYS_H = 1250  # 高边对应 x 坐标

os.makedirs(WATCH_DIR, exist_ok=True)
os.makedirs(RESULT_DIR, exist_ok=True)

# ----------------- 加载模型 -----------------
model = YOLO(MODEL_PATH)

# ----------------- 事件处理 -----------------
class NewImageHandler(FileSystemEventHandler):
    def on_created(self, event):
        if not event.is_directory and event.src_path.lower().endswith(('.jpg', '.png', '.jpeg')):
            time.sleep(0.5)  # 等待文件写入完成
            print(f"\n🖼 检测新图片: {event.src_path}")
            self.run_yolo(event.src_path)

    def run_yolo(self, img_path):
        results = model.predict(source=img_path, task="obb", save=False)
        img_h, img_w = results[0].orig_shape[:2]

        for r in results:
            if r.obb is not None and len(r.obb.xywhr) > 0:
                for box in r.obb.xywhr.cpu().numpy():
                    cx, cy, w, h, angle = box
                    # 转换为物理坐标，右上角为原点
                    phys_x = (cx / img_h) * PHYS_H
                    phys_y = ((img_w - cy) / img_w) * PHYS_W
                    print(f"像素坐标: cx={cx:.1f}, cy={cy:.1f}")
                    print(f"物理坐标: x={phys_x:.1f}mm, y={phys_y:.1f}mm, w={w:.1f}, h={h:.1f}, angle={math.degrees(angle):.1f}°")
            else:
                print("No OBB detected")

        # 保存绘制结果图像
        plotted_img = results[0].plot()  # 返回 np.ndarray
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(RESULT_DIR, f"result_{timestamp}.jpg")
        cv2.imwrite(save_path, plotted_img)
        print(f"✅ 已保存检测结果图像: {save_path}")

# ----------------- 主程序 -----------------
if __name__ == "__main__":
    event_handler = NewImageHandler()
    observer = Observer()
    observer.schedule(event_handler, WATCH_DIR, recursive=False)
    observer.start()
    print(f"📂 监听目录: {WATCH_DIR}，有新图片将自动检测并保存结果到 {RESULT_DIR} ...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
