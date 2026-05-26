import numpy as np
depth = np.load(r"D:\examples_project\python projects\depth\depth_1.npy")
depth_value = depth[722, 717]  # 获取任意像素点深度
import numpy as np


# 查看形状
print("数组形状：", depth.shape)

# 查看数组维度
print("数组维度：", depth.ndim)

# 查看数组数据类型
print("数组类型：", depth.dtype)

print(1198 - depth_value)
