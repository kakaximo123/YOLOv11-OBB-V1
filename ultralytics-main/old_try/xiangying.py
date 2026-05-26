from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient('169.254.0.66', port=502)
if client.connect():
    print("✅ 已连接")
    test = client.read_holding_registers(address=0, count=2, slave=1)
    print("📡 响应对象:", test)
    if not test.isError():
        print("📊 寄存器数据:", test.registers)
    else:
        print("❌ 读取失败:", test)
else:
    print("❌ 无法连接控制端")
client.close()
