# 8电机控制
std_msgs/Header header
# [左前轮角度,右前轮角度，左后轮角度，右后轮角度]
float64[] position
# [左前轮速度，右前轮速度，左后轮速度，右后轮速度]
float64[] speed
# 轮毂电机反馈电流，单位 A
float64[] wheel_motor_current
# 转向电机反馈电流，单位 A
float64[] steering_motor_current
# 电源板整机过流原始值
int32 robot_overcurrent