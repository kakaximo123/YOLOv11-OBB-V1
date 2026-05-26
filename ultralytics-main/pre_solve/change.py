import os
from PIL import Image  # 使用 Pillow 库

def convert_bmp_to_jpg(input_folder: str, output_folder: str, jpeg_quality: int = 95):
    """
    将 input_folder 中所有 .bmp 文件转换为 .jpg，保存到 output_folder。
    参数：
        input_folder: 源文件夹路径
        output_folder: 转换后保存的文件夹路径（若不存在会创建）
        jpeg_quality: JPEG 保存质量（0–100），默认为 95
    """
    os.makedirs(output_folder, exist_ok=True)
    for fname in os.listdir(input_folder):
        if fname.lower().endswith(".bmp"):
            bmp_path = os.path.join(input_folder, fname)
            try:
                img = Image.open(bmp_path)
                # 转换为 RGB 模式（有时候 BMP 可能是不同模式）
                rgb = img.convert("RGB")
                base = os.path.splitext(fname)[0]
                jpg_fname = base + ".jpg"
                jpg_path = os.path.join(output_folder, jpg_fname)
                rgb.save(jpg_path, "JPEG", quality=jpeg_quality)
                print(f"转换成功：{bmp_path} → {jpg_path}")
            except Exception as e:
                print(f"错误：无法转换 {bmp_path}，原因：{e}")

    print("批量转换完成。")

# 使用示例
if __name__ == "__main__":
    input_folder = r"F:\remali"
    output_folder = r"F:\mali"
    convert_bmp_to_jpg(input_folder, output_folder, jpeg_quality=95)
