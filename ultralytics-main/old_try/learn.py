# 导入所需的库
import os  # 用于文件和目录操作
import time  # 用于时间相关操作
import math  # 用于数学计算
import struct  # 用于处理二进制数据与Python数据类型的转换
from watchdog.observers import Observer  # 用于监控文件系统变化
from watchdog.events import FileSystemEventHandler  # 用于处理文件系统事件
from ultralytics import YOLO  # 导入YOLO目标检测模型
import cv2  # 用于图像处理
from pymodbus.client import ModbusTcpClient  # 用于Modbus TCP通信
import numpy as np  # 用于数值计算
from chen.Python.get_height1 import CameraHandler

# ----------------- 配置参数区域 -----------------
# 监控的图片目录，新图片会被自动检测
WATCH_DIR = r'./camera/photo'
# 检测结果保存目录
SAVE_DIR = r'./camera/detect'
# YOLO模型权重文件路径
MODEL_PATH = r'D:\examples_project\python projects\ultralytics-main\ultralytics-main\project\pillow4\weights\best.pt'

# 创建目录（如果不存在）
os.makedirs(WATCH_DIR, exist_ok=True)  # exist_ok=True表示目录存在也不报错
os.makedirs(SAVE_DIR, exist_ok=True)

# 物理尺寸参数（单位：毫米），根据实际场景调整
#PHYS_W = 60.1  # 物理宽度
#PHYS_H = 74.0  # 物理高度
Z_HEIGHT = 20.0  # Z轴高度（固定值）

h=2048

# ==== ROI 定义 ====
roi_x_left = 20
roi_x_right = 1660
roi_w_px = roi_x_right - roi_x_left
#roi = img[:, roi_x_left:roi_x_right].copy()

# ✅✅ 相机1尺度 ✅✅
real_width_cm = 60.1
real_height_cm = 74.0
pixels_per_cm_x = roi_w_px / real_width_cm
pixels_per_cm_y = h / real_height_cm

# ✅✅深度相机尺度✅✅
pixels_per_mmx = 59.8 / 515 * 10
pixels_per_mmy = 50 / 435 * 10

# 机械臂Modbus通信配置
# ROBOT_IP = "192.168.0.30"  # 机械臂IP地址（备用）
ROBOT_IP = "169.254.0.66"  # 机械臂当前使用的IP地址
ROBOT_PORT = 502  # Modbus默认端口号
# 寄存器地址定义（与机械臂通信协议对应）
#REG_X = 0  # X坐标寄存器起始地址
#REG_Y = 2  # Y坐标寄存器起始地址
#REG_Z = 4  # Z坐标寄存器起始地址
#REG_A = 6  # 角度寄存器起始地址
#REG_zw = 8  # 动作信号寄存器起始地址

REG_X, REG_Y, REG_Z = 0, 2, 4
REG_SPEED, REG_RY, REG_A = 6, 8, 10
REG_zw, REG_10_FLAG, REG_11_FLAG, REG_12_FLAG = 12, 14, 16, 18

FLOAT_ORDER = "BADC"  # 浮点数在寄存器中的字节顺序

object_count = 0

# ----------------- Modbus TCP通信相关函数 -----------------
def float_to_regs(value, order="BADC"):
    """
    将浮点数转换为Modbus寄存器值（两个16位整数）
    :param value: 要转换的浮点数
    :param order: 字节顺序
    :return: 转换后的两个寄存器值
    """
    # 将浮点数打包为4字节的二进制数据（大端模式）
    b = struct.pack('>f', value)
    # 定义不同字节顺序的映射关系
    byte_map = {
        "ABCD": [0, 1, 2, 3],
        "DCBA": [3, 2, 1, 0],
        "BADC": [1, 0, 3, 2],
        "CDAB": [2, 3, 0, 1]
    }
    # 按照指定顺序重新排列字节
    b_ordered = bytes([b[i] for i in byte_map[order]])
    # 将4字节数据解包为两个16位整数（寄存器值）
    return struct.unpack('>HH', b_ordered)


def send_position(client, x_mm, y_mm, z_mm, angle_deg, float_order=FLOAT_ORDER, pulse_zw=True):
    """
    发送坐标给机械臂，同时产生动作信号短脉冲
    :param client: Modbus客户端对象
    :param x_mm: X坐标（毫米）
    :param y_mm: Y坐标（毫米）
    :param z_mm: Z坐标（毫米）
    :param angle_deg: 角度（度）
    :param float_order: 浮点数字节顺序
    :param pulse_zw: 是否发送动作信号脉冲
    """
    # 将各坐标值转换为寄存器值
    x_regs = float_to_regs(x_mm, order=float_order)
    y_regs = float_to_regs(y_mm, order=float_order)
    z_regs = float_to_regs(z_mm, order=float_order)
    a_regs = float_to_regs(angle_deg, order=float_order)
    zw_regs_zero = float_to_regs(0.0, order=float_order)

    # 先写入坐标值和0值的动作信号
    client.write_registers(REG_X, list(x_regs))  # 写入X坐标寄存器
    client.write_registers(REG_Y, list(y_regs))  # 写入Y坐标寄存器
    client.write_registers(REG_Z, list(z_regs))  # 写入Z坐标寄存器
    client.write_registers(REG_A, list(a_regs))  # 写入角度寄存器
    client.write_registers(REG_zw, list(zw_regs_zero))  # 写入0值到动作信号寄存器

    if pulse_zw:
        # 发送动作信号短脉冲（置1）
        zw_regs_one = float_to_regs(1.0, order=float_order)
        client.write_registers(REG_zw, list(zw_regs_one))
        print(f"✅ 坐标已发送，REG_zw置1: X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f}, A={angle_deg:.1f}")
        time.sleep(0.1)  # 保持100ms的高电平
        # 恢复动作信号为0
        client.write_registers(REG_zw, list(zw_regs_zero))
        print("🔹 REG_zw已恢复为0")
    else:
        print(f"✅ 坐标已发送: X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f}, A={angle_deg:.1f}, REG_zw=0")


# ----------------- 加载YOLO模型 -----------------
model = YOLO(MODEL_PATH)  # 加载预训练的YOLO模型


# ----------------- 文件系统事件处理 -----------------
class NewImageHandler(FileSystemEventHandler):
    """处理新图片文件创建事件的类"""
    global Z_HEIGHT  # 引用全局变量Z_HEIGHT

    def __init__(self, modbus_client):
        """初始化方法"""
        super().__init__()  # 调用父类初始化方法
        self.client = modbus_client  # 保存Modbus客户端对象

    def on_created(self, event):
        """当有新文件创建时调用"""
        # 判断是否为图片文件（忽略目录和非图片文件）
        if not event.is_directory and event.src_path.lower().endswith(('.jpg', '.png', '.jpeg')):
            time.sleep(0.5)  # 等待文件完全写入
            print(f"\n🖼 检测新图片: {event.src_path}")
            self.run_yolo(event.src_path)  # 对新图片进行YOLO检测

    def run_yolo(self, img_path):
        """运行YOLO模型进行目标检测"""
        # 执行目标检测（obb表示定向边界框检测，适用于旋转目标）
        results = model.predict(source=img_path, task="obb", save=False)
        # 获取原图尺寸
        img_h, img_w = results[0].orig_shape[:2]
        # 复制原图用于绘制检测结果
        img_plot = results[0].orig_img.copy()

        # 绘制自定义坐标轴（便于理解坐标转换）
        origin = (img_w - 1, 0)  # 原点位置
        # 绘制X轴（红色）
        cv2.line(img_plot, origin, (img_w - 1, img_h - 1), (0, 0, 255), 2)
        # 绘制Y轴（蓝色）
        cv2.line(img_plot, origin, (0, 0), (255, 0, 0), 2)
        # 添加轴标签
        cv2.putText(img_plot, 'X', (img_w - 15, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img_plot, 'Y', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        any_detected = False  # 标记是否检测到目标

        # 处理检测结果
        for r in results:
            # 检查是否有定向边界框检测结果
            if r.obb is not None and len(r.obb.xywhr) > 0:
                any_detected = True  # 标记已检测到目标
                # 遍历每个检测框
                for idx, box in enumerate(r.obb.xywhr.cpu().numpy(), start=1):
                    # 解析检测框参数：中心x、中心y、宽度、高度、旋转角度（弧度）
                    cx, cy, w, h, angle = box


                    # 将图像坐标转换为物理坐标（毫米）
                    # phys_x = (0 - cy / img_h) * real_height_cm
                    # phys_y = ((img_w - cx) / img_w) * real_width_cm - 710  # -710是偏移校准值
                    dx_mm =-(cy / pixels_per_cm_y) * 10
                    cx_mm = -(cy / pixels_per_cm_y) * 10 + 340
                    cy_mm = (roi_x_right - cx) / pixels_per_cm_x * 10
                    print(cx_mm, cy_mm,angle)

                    # 计算旋转矩形的四个顶点（用于绘制）
                    pts = cv2.boxPoints(((cx, cy), (w, h), math.degrees(angle)))
                    pts = pts.astype(int)  # 转换为整数坐标
                    # 绘制检测框（绿色）
                    cv2.drawContours(img_plot, [pts], 0, (0, 255, 0), 2)
                    # 绘制中心点（红色）
                    cv2.circle(img_plot, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    # 标记检测框序号
                    cv2.putText(img_plot, f"#{idx}", (int(cx) + 5, int(cy) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # 绘制YOLO水平线（青色）
                    cv2.line(img_plot, (0, int(cy)), (img_w - 1, int(cy)), (255, 255, 0), 1, lineType=cv2.LINE_AA)
                    cv2.putText(img_plot, 'YOLO Horizontal', (10, int(cy) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                    # 计算长边角度（更准确的目标朝向）
                    # 获取四个边的端点
                    edges = [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
                    # 计算每条边的长度
                    edge_lengths = [np.linalg.norm(np.array(p2) - np.array(p1)) for p1, p2 in edges]
                    # 找到最长边的索引
                    longest_idx = np.argmax(edge_lengths)
                    p1, p2 = edges[longest_idx]  # 最长边的两个端点
                    # 计算长边的方向向量
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    # 计算角度（弧度转角度）
                    final_angle = math.degrees(math.atan2(dy, dx))

                    # 角度调整（确保在0-180度范围内）
                    if final_angle < 0:
                        final_angle = 180 + final_angle

                    # 绘制最长边（黄色）
                    cv2.line(img_plot, tuple(p1), tuple(p2), (0, 255, 255), 2)
                    # 显示角度值
                    cv2.putText(img_plot, f"Angle: {final_angle:.1f}", (int(cx), int(cy) - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    #1
                    shendu_location = (1010 - int(cy_mm / pixels_per_mmx), 200 + int(-dx_mm / pixels_per_mmy))  # 计算深度查询坐标


                    cz_mm = cam.get_height(shendu_location[0], shendu_location[1])  # 同步调用深度相机获取高度

                    # now2 = time.time()
                    cz_mm = 1190 - cz_mm  # 深度值校准（根据实际场景调整）

                    cz_mm = cz_mm - 50

                    if cz_mm <= 10 :
                        cz_mm = 1190
                        print('深度出现负值')

                    #2
                    # object_count =+ 1
                    # shendu_location = (1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))
                    # cam.save_depth_frame(object_count)
                    # depth = np.load(f"D:\examples_project\python projects\depth\depth_{object_count}.npy")
                    # depth_value = depth[shendu_location[0] + 4, shendu_location[1] + 4]  # 获取任意像素点深度
                    # cz_mm = 1198 - depth_value


                    # 发送坐标给机械臂
                    send_position(self.client, cx_mm, cy_mm, cz_mm, final_angle, pulse_zw=True)
                    print(f"⏱ 检测框 {idx} 坐标已发送，等待 20 秒再发送下一个目标...")
                    time.sleep(5)  # 测试用20秒，正式环境可改为300秒

        # 如果没有检测到任何目标，发送0值保持REG_zw为0
        if not any_detected:
            send_position(self.client, 0, 0, 0, 0, pulse_zw=False)

        # 保存检测结果图片
        filename = os.path.basename(img_path)  # 获取原文件名
        save_path = os.path.join(SAVE_DIR, f"detect_{filename}")  # 构造保存路径
        cv2.imwrite(save_path, img_plot)  # 保存图片
        print(f"✅ 检测结果已保存: {save_path}")


# ----------------- 主程序入口 -----------------
if __name__ == "__main__":
    # 创建Modbus TCP客户端并连接机械臂
    cam = CameraHandler()
    cam.open_camera()
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT)
    if not client.connect():
        print("❌ 连接机械臂失败")
        exit()  # 连接失败则退出程序
    print("✅ 成功连接机械臂")

    # 创建事件处理器（传入Modbus客户端）
    event_handler = NewImageHandler(client)
    # 创建文件系统监控器
    observer = Observer()
    # 配置监控器：监控WATCH_DIR目录，不递归子目录
    observer.schedule(event_handler, WATCH_DIR, recursive=False)
    # 启动监控器
    observer.start()
    print(f"📂 监听目录: {WATCH_DIR}，有新图片将自动检测并发送机械臂坐标...")

    try:
        # 保持程序运行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断信号，停止监控器
        observer.stop()
    # 等待监控器线程结束
    observer.join()
    # 关闭Modbus连接
    client.close()