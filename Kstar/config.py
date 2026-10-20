"""KSG1-250K Modbus RTU 仿真的通信与读取配置。"""

# 端口号是各自机器的本地 COM 端口。
SLAVE_A_PORT = "COM3"
SLAVE_B_PORT = "COM4"
MASTER_PORT = "COM5"

# 《KSG1-250K 逆变器 MODBUS 通讯协议 V3.6》规定的默认串口格式。
BAUDRATE = 9600
PARITY = "N"
BYTESIZE = 8
STOPBITS = 1

# Modbus RTU 从站地址。协议允许范围为 0–320；本仿真使用地址 1。
SLAVE_ID = 1
# 有线 RS-485 用 3 秒足够；无线透传链路延迟大且不稳定，放宽到 10 秒。
RESPONSE_TIMEOUT = 10

# 每次读请求之间的间隔（秒）。无线透传链路上一笔事务的旧响应可能
# 延迟或重复送达，留出间隔并配合接收缓冲清理，避免旧帧污染下一笔事务。
INTER_REQUEST_DELAY = 1.0

# 每个读请求的尝试次数。读操作幂等，无线链路偶发丢响应时自动重发同一条请求。
READ_ATTEMPTS = 3

# 协议功能码。03H 读取 Holding Registers；04H 读取 Input Registers。
READ_HOLDING_REGISTERS = 0x03
READ_INPUT_REGISTERS = 0x04

# Modbus 读寄存器请求每帧最多可读取 125 个 16-bit registers。
MAX_REGISTERS_PER_REQUEST = 125

# 文档 Register Address 直接作为 Modbus PDU 地址使用。
# PyModbus 3.3 服务端会在数据块访问时自动加 1，数据块生成器统一处理该偏移。
PYMODBUS_33_SERVER_ADDRESS_OFFSET = 1
