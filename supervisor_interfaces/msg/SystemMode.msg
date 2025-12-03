# System Mode Message
# 用于请求或发布系统运行模式

# Mode values
uint8 UNDEFINED=0       # 未定义
uint8 AUTONOMOUS=1      # 自动
uint8 TELEOPERATION=2   # 遥操
uint8 MAPPING=3         # 建图
uint8 STANDBY=4         # 待命, 所有服务停止

# 当前/请求的模式值
uint8 mode
string mode_name
