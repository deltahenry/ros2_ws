from pymodbus.client import ModbusTcpClient

# 設備參數
ip = "192.168.1.10"           # 請換成你的設備 IP
port = 502                    # Modbus TCP 默認通訊埠
slave_id = 2                   # 你的 Slave ID
register_address = 0X9C60     # 要寫入的暫存器地址
value_to_write = 0        # 寫入的值（16-bit 整數）

# 建立連線
client = ModbusTcpClient(ip, port=port)
client.connect()

# 寫入單一暫存器
result = client.write_register(address=register_address, value=value_to_write, slave=slave_id)

# 檢查是否成功
if not result.isError():
    print(f"✅ 成功寫入 Slave {slave_id} 的暫存器 {register_address}，值：{value_to_write}")
else:
    print(f"❌ 寫入失敗：{result}")

# 關閉連線
client.close()