# 关节运行状态枚举类型
uint8 IDLE = 0       # 位置到位停止
uint8 RUNNING = 1    # 正在展开/抓取
uint8 STALL = 2      # 电机堵转停止
uint8 ERROR = 3      # 电机报错（读取error_code）
uint8 UNKNOWN = 255  # 未知异常