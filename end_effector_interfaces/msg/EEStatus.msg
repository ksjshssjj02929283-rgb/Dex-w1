# 末端执行器运行状态枚举类型
uint8 IDLE = 0       # 位置到位停止（所有关节都IDLE）
uint8 RUNNING = 1    # 正在展开/抓取（至少一个关节RUNNING）
uint8 ERROR = 2      # 电机报错（至少一个关节ERROR）
uint8 UNCONNECT = 100  # 串口未连接
uint8 UNKNOWN = 255  # 未知异常（至少一个关节UNKNOWN）