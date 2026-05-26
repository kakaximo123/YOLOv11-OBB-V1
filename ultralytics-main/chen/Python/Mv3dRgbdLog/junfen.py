import os
import shutil

def split_images_into_four_parts(src_folder):
    # 获取所有图片文件
    images = [f for f in os.listdir(src_folder)
              if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"))]

    total = len(images)
    print(f"总图片数: {total}")

    # 按顺序排序（保证可重复）
    images.sort()

    # 计算每份大小
    part_size = total // 4
    remainder = total % 4  # 599 % 4 = 3，用来补到前几份

    start = 0
    for i in range(1, 5):
        # 每份的最终大小
        size = part_size + (1 if i <= remainder else 0)
        end = start + size

        # 创建子文件夹
        folder_name = os.path.join(src_folder, f"part_{i}")
        os.makedirs(folder_name, exist_ok=True)

        # 移动文件
        for img in images[start:end]:
            shutil.move(os.path.join(src_folder, img),
                        os.path.join(folder_name, img))

        print(f"第 {i} 份: {size} 张 -> {folder_name}")

        start = end


if __name__ == "__main__":
    src = r"D:\photo"  # ← 修改成你的目录
    split_images_into_four_parts(src)
