from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
import struct, time, threading

# =====================
# 工具函数
# =====================
def float_to_regs(value, order="BADC"):
    """浮点数 -> 2个寄存器（BADC 顺序）"""
    b = struct.pack('>f', float(value))
    if order == "BADC":
        b = b[1:2] + b[0:1] + b[3:4] + b[2:3]
    elif order == "CDAB":
        b = b[2:4] + b[0:2]
    regs = struct.unpack('>HH', b)
    return list(regs)

def regs_to_float(regs, order="BADC"):
    """2寄存器 -> 浮点数"""
    if len(regs) != 2:
        raise ValueError("需要两个寄存器")
    b = struct.pack('>HH', *regs)
    if order == "BADC":
        b = b[2:4] + b[0:2]
    elif order == "CDAB":
        b = b[::-1]
    return struct.unpack('>f', b)[0]


# =====================
# 初始化寄存器数据块
# =====================
store = ModbusSlaveContext(
    hr=ModbusSequentialDataBlock(0, [0]*120)    # 预留200个寄存器
)
context = ModbusServerContext(slaves=store, single=True)


# =====================
# 电脑 → 工控机（工控机读 0 起始的寄存器）
# =====================
def update_data(context):
    """周期性更新供工控机读取的数据（寄存器 0 起）"""
    i = 0
    while True:
        time.sleep(1)
        slave_id = 0x01
        address = 0  # ✅ 工控机03读取起始地址
        values = [i*0.1, i*0.2, i*0.3, i*0.4, i*0.5, i*0.6]
        regs = []
        for v in values:
            regs.extend(float_to_regs(v, order="BADC"))
        context[slave_id].setValues(3, address, regs)
        i += 1  # 不打印发送


# =====================
# 工控机 → 电脑（工控机16写 40 起始的寄存器）
# =====================
def monitor_data(context):
    """监控工控机写入寄存器 40~59 的数据"""
    last_vals = None
    while True:
        time.sleep(0.5)
        slave_id = 0x01
        address = 40  # ✅ 工控机16写起始地址
        count = 20
        regs = context[slave_id].getValues(3, address, count=count)
        vals = []
        for i in range(0, len(regs), 2):
            vals.append(regs_to_float(regs[i:i+2], order="BADC"))

        # ✅ 仅当工控机写入变化时才打印
        if vals != last_vals:
            print(f"📩 收到工控机写入（地址40~59）数据：{[round(v,3) for v in vals]}")
            last_vals = vals


# =====================
# 主函数：启动 Modbus TCP 从站
# =====================
if __name__ == "__main__":
    print("🚀 启动 Modbus TCP 从站服务器，等待工控机（主站）连接...")
    threading.Thread(target=update_data, args=(context,), daemon=True).start()
    threading.Thread(target=monitor_data, args=(context,), daemon=True).start()
    StartTcpServer(context, address=("0.0.0.0", 502))



