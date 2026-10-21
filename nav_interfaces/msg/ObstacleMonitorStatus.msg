# collision_monitor 障碍检测实时状态：每个处理周期发布一次（非仅状态切换时）。
std_msgs/Header header

uint8 status
uint8 FREE = 0
uint8 SLOWDOWN = 1
uint8 STOP = 2

string zone_name        # 触发该状态的多边形/区域名，FREE 时为空
float32 slowdown_ratio  # SLOWDOWN 时相对当前速度的限速比例(0~1)，其余状态为 0

# 停止 / 降速原因（STOP 时区分是障碍物、传感器失效还是外轮廓缺失；FREE 时为 REASON_NONE）
uint8 reason
uint8 REASON_NONE = 0
uint8 REASON_OBSTACLE = 1
uint8 REASON_SENSOR_FAULT = 2
uint8 REASON_FOOTPRINT_MISSING = 3

string fault_source     # REASON_SENSOR_FAULT 时为失效的数据源名，其余为空
