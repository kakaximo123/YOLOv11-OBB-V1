import os


def rename_bmp_files(folder_path, prefix="", start_index=1, keep_extension=True):
    """
    将指定文件夹中所有 .bmp 文件按顺序重命名。

    参数：
    - folder_path: 要操作的文件夹路径（字符串）
    - prefix: 重命名前缀（字符串），例如 "image_" 会变成 image_1.bmp
    - start_index: 起始编号（整数），默认为 1
    - keep_extension: 是否保留原扩展名（布尔），如果 True，会变成 prefix1.bmp；如果 False，可以改成例如 .png 等

    注意：该函数只重命名 .bmp 文件，不会处理子文件夹中的文件。
    """
    # 获取文件列表（只当前文件夹，不递归子目录）
    files = os.listdir(folder_path)
    # 过滤出 .bmp 文件（不区分大小写扩展名）
    bmp_files = [f for f in files if f.lower().endswith(".bmp")]
    # 可按名字排序（如果你希望按照原名字顺序重命名）
    bmp_files.sort()

    current_index = start_index
    for fname in bmp_files:
        old_path = os.path.join(folder_path, fname)
        # 构造新文件名
        if keep_extension:
            new_fname = f"{prefix}{current_index}.bmp"
        else:
            # 如果你想改成其它扩展，比如 .png
            new_fname = f"{prefix}{current_index}.png"
        new_path = os.path.join(folder_path, new_fname)

        # 避免命名冲突：如果 new_path 已经存在，可以先报错或跳过
        if os.path.exists(new_path):
            print(f"警告：目标文件已存在，跳过重命名 {old_path} → {new_path}")
        else:
            os.rename(old_path, new_path)
            print(f"重命名：{old_path} → {new_path}")

        current_index += 1

    print("重命名完成，共处理", len(bmp_files), "个 .bmp 文件。")


# 使用示例
if __name__ == "__main__":
    folder = r"F:\remali"
    rename_bmp_files(folder_path=folder, prefix="", start_index=1, keep_extension=True)
