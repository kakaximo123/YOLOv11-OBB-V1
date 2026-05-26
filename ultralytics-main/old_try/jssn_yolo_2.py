import os
import time
import math
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from ultralytics import YOLO
import cv2

# ----------------- 配置 -----------------
WATCH_DIR = r'./camera/photo'  # 监听截图保存目录
SAVE_DIR = r'./camera/detect'  # 检测结果保存目录
MODEL_PATH = r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\runs\obb\train10\weights\best.pt'

os.makedirs(WATCH_DIR, exist_ok=True)
os.makedirs(SAVE_DIR, exist_ok=True)

# 图片物理尺寸（单位 mm）
PHYS_W = 1750  # 宽边对应 y 坐标
PHYS_H = 1250  # 高边对应 x 坐标

# 加载 YOLO 模型
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

        # 将原图转换为可绘制的 BGR numpy 数组
        img_plot = results[0].orig_img.copy()

        # 绘制坐标轴
        origin = (img_w - 1, 0)  # 右上角为原点
        cv2.line(img_plot, origin, (img_w - 1, img_h - 1), (0, 0, 255), 2)  # x轴（纵向递减）
        cv2.line(img_plot, origin, (0, 0), (255, 0, 0), 2)  # y轴（横向增大）
        cv2.putText(img_plot, 'X', (img_w - 15, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img_plot, 'Y', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # 输出检测结果坐标信息
        for r in results:
            if r.obb is not None and len(r.obb.xywhr) > 0:
                for box in r.obb.xywhr.cpu().numpy():
                    cx, cy, w, h, angle = box

                    # 物理坐标计算
                    phys_x = (0 - cy / img_h) * PHYS_H  # x轴从上到下递减，可为负
                    phys_y = ((img_w - cx) / img_w) * PHYS_W  # y轴从右向左增大
                    phys_y = phys_y - 715

                    print(f"像素坐标: cx={cx:.1f}, cy={cy:.1f}")
                    print(f"物理坐标: x={phys_x:.1f}mm, y={phys_y:.1f}mm, angle={math.degrees(angle):.1f}°")

                    # 在图像上标注质心
                    cx_int, cy_int = int(cx), int(cy)
                    cv2.circle(img_plot, (cx_int, cy_int), 5, (0, 0, 255), -1)  # 红色圆点表示质心
                    cv2.putText(
                        img_plot,
                        f"({phys_x:.0f},{phys_y:.0f})",
                        (cx_int + 5, cy_int - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )
            else:
                print("No OBB detected")

        # 保存带检测框和质心的图像
        filename = os.path.basename(img_path)
        save_path = os.path.join(SAVE_DIR, f"detect_{filename}")
        cv2.imwrite(save_path, img_plot)
        print(f"✅ 检测结果已保存: {save_path}")


# ----------------- 主程序 -----------------
if __name__ == "__main__":
    event_handler = NewImageHandler()
    observer = Observer()
    observer.schedule(event_handler, WATCH_DIR, recursive=False)
    observer.start()
    print(f"📂 监听目录: {WATCH_DIR}，有新图片将自动进行检测并保存结果...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
