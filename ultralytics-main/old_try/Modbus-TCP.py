import struct
from pymodbus.client import ModbusTcpClient

# ========================
#   Modbus 连接配置
# ========================
ROBOT_IP = "192.168.0.66"
ROBOT_PORT = 502

REG_X = 0
REG_Y = 2
REG_A = 8

# ========================
#   浮点格式转换
# ========================
def float_to_regs(value, order="BADC"):
    """
    将32位浮点数转为两个16位寄存器
    order: 寄存器字节顺序
        "ABCD" = big-endian
        "DCBA" = little-endian
        "BADC" = big-endian + byte swap (你现在的情况)
        "CDAB" = little-endian + byte swap
    """
    # 按大端模式打包成4字节 (ABCD)
    b = struct.pack('>f', value)
    byte_map = {
        "ABCD": [0, 1, 2, 3],
        "DCBA": [3, 2, 1, 0],
        "BADC": [1, 0, 3, 2],
        "CDAB": [2, 3, 0, 1]
    }
    if order not in byte_map:
        raise ValueError(f"不支持的寄存器顺序: {order}")

    # 重新排列字节
    b_ordered = bytes([b[i] for i in byte_map[order]])
    # 转成两个16位寄存器
    return struct.unpack('>HH', b_ordered)

# ========================
#   发送坐标函数
# ========================
def send_position(client, x_cm, y_cm, angle_deg, float_order="BADC"):
    """发送质心位置和角度到机械臂"""
    x_mm = x_cm * 10
    y_mm = y_cm * 10
    x_regs = float_to_regs(x_mm, order=float_order)
    y_regs = float_to_regs(y_mm, order=float_order)
    a_regs = float_to_regs(angle_deg, order=float_order)

    client.write_registers(REG_X, list(x_regs))
    client.write_registers(REG_Y, list(y_regs))
    client.write_registers(REG_A, list(a_regs))
    print(f"已发送: X={x_mm:.1f}mm, Y={y_mm:.1f}mm, A={angle_deg:.1f}° (格式: {float_order})")

# ========================
#   连接机械臂并发送
# ========================
if __name__ == "__main__":
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT)
    if not client.connect():
        print("连接机械臂失败，请检查IP/端口/网络设置")
        exit()
    else:
        print("成功连接机械臂")

    # 发送测试坐标，格式为 BADC（Big-endian + byte swap）
    for i in range(100000):
        send_position(client, -86.8, 20.9, 20, float_order="BADC")

    client.close()



# for i in range(100000):
#     send_position(client, 3.4, 6.5, 100.0)
#     print("over!")

# # ========================
# #   摄像头 / 图像处理
# # ========================
# cap = cv2.VideoCapture(0)  # 0 为默认摄像头
# display = "检测窗口"
# cv2.namedWindow(display, cv2.WINDOW_NORMAL)
# cv2.resizeWindow(display, 1280, 720)
#
# real_width_cm = 60.0
# real_height_cm = 61.3
#
# while True:
#     ret, img = cap.read()
#     if not ret:
#         break
#
#     h, w = img.shape[:2]
#     roi_w = int(w * 9 / 11)
#     roi = img[:, :roi_w].copy()
#
#     pixels_per_cm_x = roi_w / real_width_cm
#     pixels_per_cm_y = h / real_height_cm
#
#     gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
#     edges = cv2.Canny(gray, 20, 25)
#     kernel = np.ones((3, 3), np.uint8)
#     edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
#
#     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#
#     for cnt in contours:
#         if cv2.contourArea(cnt) < 20000:
#             continue
#
#         rect = cv2.minAreaRect(cnt)  # (中心, (宽,高), 角度)
#         angle = rect[2]
#         width, height = rect[1]
#
#         if width < height:
#             angle += 90
#             main_edge_indices = (1, 2)
#         else:
#             main_edge_indices = (0, 1)
#
#         box = cv2.boxPoints(rect)
#         box = np.int0(box)
#
#         pt1 = tuple(box[main_edge_indices[0]])
#         pt2 = tuple(box[main_edge_indices[1]])
#
#         cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
#         cv2.line(img, pt1, pt2, (0, 255, 0), 2)
#
#         M = cv2.moments(cnt)
#         if M["m00"] != 0:
#             cx_px = int(M["m10"] / M["m00"])
#             cy_px = int(M["m01"] / M["m00"])
#
#             cx_cm = (roi_w - cx_px) / pixels_per_cm_x
#             cy_cm = cy_px / pixels_per_cm_y
#
#             cv2.circle(img, (cx_px, cy_px), 4, (0, 0, 255), -1)
#             label = f"({cx_cm:.1f}cm, {cy_cm:.1f}cm, {angle:.1f}°)"
#             cv2.putText(img, label, (cx_px + 10, cy_px - 10),
#                         cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 255, 0), 1)
#
#             # 发送给机械臂
#             send_position(client, cx_cm, cy_cm, angle)
#
#     cv2.line(img, (0, h - 50), (roi_w, h - 50), (255, 0, 0), 2)  # 底部水平基准线
#     cv2.imshow(display, img)
#
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#
# cap.release()
# cv2.destroyAllWindows()
# 示例发送
# send_position(client, 76.2, 38.4, 45.0)
client.close()
