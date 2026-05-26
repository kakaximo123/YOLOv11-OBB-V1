import cv2
import os

# 鼠标回调函数
ref_point = []
cropping = False

def click_and_crop(event, x, y, flags, param):
    global ref_point, cropping, image, orig

    if event == cv2.EVENT_LBUTTONDOWN:
        ref_point = [(x, y)]
        cropping = True

    elif event == cv2.EVENT_MOUSEMOVE:
        if cropping:
            img_copy = orig.copy()
            cv2.rectangle(img_copy, ref_point[0], (x, y), (0, 255, 0), 2)
            cv2.imshow("image", img_copy)

    elif event == cv2.EVENT_LBUTTONUP:
        ref_point.append((x, y))
        cropping = False

        cv2.rectangle(image, ref_point[0], ref_point[1], (0, 255, 0), 2)
        cv2.imshow("image", image)

        x1, y1 = ref_point[0]
        x2, y2 = ref_point[1]
        print(f"选区坐标：({x1}, {y1}) --> ({x2}, {y2})")

# 请修改此处为你的 BMP 文件路径
img_path = r"C:\Users\Administrator\HiViewer\Data\MV-DT01SDU(DA3773384)\1.bmp"
if not os.path.isfile(img_path):
    raise FileNotFoundError(f"图像文件未找到：{img_path}")

# 以默认方式读取（彩色模式）
image = cv2.imread(img_path, cv2.IMREAD_COLOR)
if image is None:
    raise ValueError(f"无法读取图像文件 (可能格式或路径问题)：{img_path}")

orig = image.copy()

cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback("image", click_and_crop)

print("请用鼠标拖动选择感兴趣区域。按 ‘c’ 裁剪保存，按 ‘r’ 重置，按 ‘q’ 退出。")

while True:
    cv2.imshow("image", image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("r"):
        image = orig.copy()
        ref_point = []
        print("已重置选区。")

    elif key == ord("c"):
        if len(ref_point) == 2:
            x1, y1 = ref_point[0]
            x2, y2 = ref_point[1]
            x_min, x_max = min(x1, x2), max(x1, x2)
            y_min, y_max = min(y1, y2), max(y1, y2)
            roi = orig[y_min:y_max, x_min:x_max]
            save_name = "cropped.bmp"  # 可以保留为 BMP 或改为 PNG/JPG
            cv2.imwrite(save_name, roi)
            print(f"已保存截取区域为 {save_name}")
            print(f"实际裁剪范围：x=[{x_min},{x_max}), y=[{y_min},{y_max}) 大小 = {x_max-x_min}×{y_max-y_min}")
        else:
            print("未选择区域，请先拖选一个矩形。")

    elif key == ord("q"):
        break

cv2.destroyAllWindows()
