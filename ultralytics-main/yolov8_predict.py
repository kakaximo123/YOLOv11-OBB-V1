from ultralytics import YOLO
import math
# 加载预训练的 YOLOv8n 模型
model = YOLO(r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\runs\obb\train23\weights\best.pt')
# model = YOLO('yolov8n.pt')
# 定义图像文件的路径
# source = (r'"D:\examples_project\train_data\images\train\225.jpg"') #更改为自己的图片路径
# 运行推理，并附加参数
# model.predict(source, save=True)
results = model.predict(source=r"D:\shuju1\1.jpg", task="obb", save=True)
print(results)

for r in results:
    if r.obb is not None and len(r.obb.xywhr) > 0:
        boxes = r.obb.xywhr.cpu().numpy()
        classes = r.obb.cls.cpu().numpy().astype(int)

        for box, cls_id in zip(boxes, classes):
            name = model.names[cls_id]   # 类别名
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


