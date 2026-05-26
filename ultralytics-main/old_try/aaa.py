import cv2
from ultralytics import YOLO
import math
# 加载你训练好的模型
model = YOLO(r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\runs\obb\train11\weights\best.pt')
kkk = r"D:\examples_project\dataset\pickzawu\images\train\34.jpg"
# 打开摄像头
cap = cv2.VideoCapture(0)  # 0 = 默认摄像头

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = kkk
    # 推理（直接传入OpenCV图像）
    results = model(frame)

    # 在图像上绘制检测框（Ultralytics自带）
    annotated_frame = results[0].plot()
    r = results[0]

    if r.obb is not None and len(r.obb.xywhr) > 0:
        boxes = r.obb.xywhr.cpu().numpy()
        classes = r.obb.cls.cpu().numpy().astype(int)

        for box, cls_id in zip(boxes, classes):
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

            print(f"{name}: cx={cx:.1f}, cy={cy:.1f}, w={w:.1f}, h={h:.1f}, 长边角度={angle_deg:.1f}°")
    else:
        print("No OBB detected")

    # 显示结果
    cv2.imshow("YOLOv11-OBB Realtime", annotated_frame)

    # 按 ESC 退出
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
