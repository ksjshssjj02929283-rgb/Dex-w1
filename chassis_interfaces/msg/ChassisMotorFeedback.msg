# 底盘电机反馈（差速 / 全向通用）
#
# 由 chassis_node 从共享内存直通发布。数组长度随底盘类型变化，
# 消费前先读 chassis_type，不要假设固定长度。
#
# 【空数组约定】本机型不提供某项数据时，对应数组**留空**（size()==0），不填 0。
# 因为 0 对错误码（无错误）和温度（0℃）都是合法值，填 0 会让消费方无法区分
# 「没有这个数据」与「数据就是 0」。消费前先查 size()。
#
# 速度字段名一律带单位后缀（speed_rpm / speed_mps），不留裸 speed——
# 本系统里 RPM 与 m/s 并存，不带单位的字段名已两次导致误读。
# 换算由底层用标定后的轮径完成：轮径是标定参数，不应泄露到上层各自换算。

std_msgs/Header header

# ===== 底盘类型 =====
# 取自 robot_config.yaml 的 chassis_config.chassis_type，运行期不变
uint8 CHASSIS_DIFF_DRIVE = 0
uint8 CHASSIS_OMNI       = 1
uint8 chassis_type

# ===== 运动反馈 =====
# 转向角度 (rad)
#   全向：4 个 [左前, 右前, 左后, 右后]
#   差速：空数组（无转向电机；勿用它塞轮子累计转角，那是另一个语义）
float64[] position

# 轮速 (RPM，驱动器原始量纲)
#   全向：4 个 [左前, 右前, 左后, 右后]
#   差速：2 个 [左, 右]
float64[] speed_rpm

# 轮速 (m/s)，由 speed_rpm 经标定轮径换算
# 与 speed_rpm 一一对应、长度相同
float64[] speed_mps

# ===== 电流反馈 =====
# 轮毂电机（驱动）反馈电流 (A)。全向 4 个；差速 2 个 [左, 右]
float64[] wheel_motor_current
# 转向电机（舵）反馈电流 (A)。全向 4 个；差速：空数组
float64[] steering_motor_current
# 电源板整机过流原始值（原始值直通，阈值判定归上层）
int32 robot_overcurrent

# ===== 诊断反馈 =====
# 驱动器状态字
uint16 status_word
# 各轮错误码。差速 2 个 [左, 右]；全向：**空数组**（硬件未提供 4 轮错误码）
uint16[] error_code
# 驱动器温度 (℃)
float64 driver_temperature
# 各轮电机温度 (℃)。差速 2 个 [左, 右]；全向：**空数组**（硬件未提供 4 轮温度）
float64[] motor_temperature

# 共享内存原始时间戳（与 header.stamp 不同源，仅用于判断数据新鲜度，勿混用）
int64 shm_timestamp
