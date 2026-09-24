# System Command Message
# 用于请求系统指令

uint8 REBOOT = 1        # 重启
uint8 POWER_OFF = 2     # 关机 弃用
uint8 CLEAR_ERROR = 3   # 清错
uint8 RESET = 4         # 复位
uint8 QUICKSTOP = 5     # 快速停止
# 请求的系统指令
uint8 command
