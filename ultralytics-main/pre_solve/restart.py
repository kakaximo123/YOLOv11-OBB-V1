import os
import cv2

def batch_crop_bmp(
    input_dir: str,
    output_dir: str,
    crop_top_left: tuple = (490, 13),
    crop_bottom_right: tuple = (1018, 1106),
    ext: str = ".bmp"
):
    """
    遍历 input_dir（含子目录）所有 .bmp 文件，
    将每张图片按 crop_top_left -> crop_bottom_right 裁剪，
    保存到 output_dir（保留子目录结构／或统一存放）中。

    参数：
        input_dir: 输入根文件夹路径
        output_dir: 输出文件夹路径（若不存在会创建）
        crop_top_left: (x, y) 左上角坐标
        crop_bottom_right: (x2, y2) 右下角坐标
        ext: 要处理的文件扩展名（小写，如 ".bmp"）
    """
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)

    x1, y1 = crop_top_left
    x2, y2 = crop_bottom_right

    count = 0
    for root, dirs, files in os.walk(input_dir):
        for fname in files:
            if fname.lower().endswith(ext):
                in_path = os.path.join(root, fname)
                # 读取图片
                img = cv2.imread(in_path, cv2.IMREAD_COLOR)
                if img is None:
                    print(f"[跳过] 无法打开图片：{in_path}")
                    continue

                # 检查图片尺寸是否足够裁剪
                h, w = img.shape[:2]
                if x2 > w or y2 > h:
                    print(f"[警告] 图片尺寸太小，跳过：{in_path} (w={w},h={h})")
                    continue

                # 裁剪
                cropped = img[y1:y2, x1:x2]  # 注意：先行（y）再列（x） :contentReference[oaicite:0]{index=0}

                # 构造保存路径：你可以保持原子目录结构或统一输出
                # 这里简单统一放在 output_dir，按原文件名保存
                base_name = os.path.splitext(fname)[0]
                out_name = f"{base_name}_crop{ext}"
                out_path = os.path.join(output_dir, out_name)

                # 保存
                cv2.imwrite(out_path, cropped)
                count += 1
                if count % 100 == 0:
                    print(f"已处理 {count} 张图片...")

    print(f"处理完成，总共裁剪保存 {count} 张图片。")

# 使用示例
if __name__ == "__main__":
    input_folder = r"F:\malibmp"
    output_folder = r"F:\remali"
    batch_crop_bmp(input_folder, output_folder)
