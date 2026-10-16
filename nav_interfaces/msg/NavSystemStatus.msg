# 导航系统聚合状态（command_board 对外发布，~1Hz）
# 各字段来源见注释；后续可扩展导航/任务进度等。
std_msgs/Header header

# ── 电池（解析自 /diagnostics_agg 的 PowerBoard 段）──
uint16  battery_percentage   # 电量百分比 0-100
bool    battery_charging     # 是否充电
float32 battery_voltage      # 电压 V
float32 battery_current      # 电流 A（负=放电）

# ── 安全状态（解析自 /diagnostics_agg 的 PowerBoard Safety 段）──
bool estop_active            # 急停触发
bool bumper_front_active     # 前防撞条触发
bool bumper_rear_active      # 后防撞条触发

# ── 定位状态（来自 /localization_node/state）──
nav_interfaces/LocalizationState localization

# ── 当前加载地图（来自 map_manager/map_name）──
string current_map_name

# ── 任务状态（来自 mission_manager/status）──
string task_status

# ── 位姿 + 速度（来自 /feedback/odom）──
geometry_msgs/Pose  pose       # odom 系位姿
geometry_msgs/Twist velocity   # 机体速度

# ── 急停丢位恢复状态（来自 SafetyReactor）──
string recover_state    # NORMAL / SAFE_STOP / RELOCATING / GLOBAL_RELOC / ALARM
string safety_alarm     # 告警信息（ALARM 时非空，否则空）

# ── 导航故障（command_board 判定，如传感器失效；无故障时为 E_CB_NONE）──
int32  nav_fault_code    # 当前导航故障码，取值见下方 E_CB_*
string nav_fault_detail  # 故障说明（如失效的传感器名），无故障时为空

# ── 传感器监测状态（command_board 上报，见 SensorState.msg）──
nav_interfaces/SensorState[] sensors

# ── 编排域状态码：模块 22 dexe_mobile_nav + 子模块 04 command_board + 序号 2 位 + 等级 1 位，共 7 位。
#    等级：0 状态反馈 / 1 一级故障 / 2 二级故障 / 3 三级故障，level = code % 10。
#    测试用 后期需要统一规划
int32 E_CB_NONE             = 0
int32 E_CB_NAV_SENSOR_FAULT = 2204182   # 导航必要传感器失效：已取消导航，传感器恢复并稳定后自动解除（等级 2）
