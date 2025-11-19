# 关节状态枚举类型

# 位置到位停止
uint8 IDLE = 0

# 正在展开/抓取
uint8 RUNNING = 1

# 电流保护停止、力控到位停止、电机堵转停止
uint8 STALL = 2

# 没连上modbus串口
uint8 UNKNOWN = 3