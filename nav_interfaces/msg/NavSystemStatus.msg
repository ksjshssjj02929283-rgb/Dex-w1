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

# ── 编排域状态码（整机状态码体系）：模块 22 dexe_mobile_nav + 子模块 04 command_board
#    + 状态序号 2 位 + 等级末位，共 7 位。
#    等级：0 状态反馈 / 1 一级故障 / 2 二级故障 / 3 三级故障，可由 level = code % 10 取出。
#    ⚠️ 划域原则（DEC-007）：按「谁产生这个错误」分配，不按功能分配。
#       建图节点产生的进建图域 2101xxx，定位节点产生的进定位域 2102xxx，
#       command_board 作为编排层产生的（含对子进程 / 下游服务失败的翻译）一律进本段。
#    align_map_tool 是 CLI，本身不带 ROS 错误码，只有 exit code + --emit_json 的 reason，
#       由 command_board 翻译成下列 E_CB_ALIGN_* 之一。
int32 E_CB_NONE                  = 0
int32 E_CB_ALIGN_MAP_NOT_FOUND   = 2204010   # 目标地图不存在 / 未加载任何地图（等级 0）
int32 E_CB_ALIGN_ARG_INVALID     = 2204020   # 入参非法（如 min_yaw_deg 为负）（等级 0）
int32 E_CB_ALIGN_BUSY            = 2204030   # 已有对齐 / 替换 / 重置原点任务在执行（等级 0；三者共用同一把互斥锁）
int32 E_CB_ALIGN_ESTIMATE_FAILED = 2204040   # dry_run 估角失败：无线特征 / 无可靠正交结构（等级 0，零文件改动）
int32 E_CB_ALIGN_TOOL_FAILED     = 2204052   # align_map_tool 子进程非零退出（等级 2）
int32 E_CB_ALIGN_BACKUP_FAILED   = 2204062   # 资产备份失败，已中止且未改任何文件（等级 2）
int32 E_CB_ALIGN_ASSETS_FAILED   = 2204072   # 资产坐标变换（transform_assets_tool）失败 / 超时，已回滚底图（等级 2；message 带回资产域原始码）
int32 E_CB_ALIGN_ROLLBACK_FAILED = 2204083   # 回滚失败：底图与资产可能不一致，需人工介入（等级 3）
int32 E_CB_LOC_STOP_TIMEOUT      = 2204092   # 停定位超时（等级 2，此时仍零文件改动；对齐 / 替换共用）
int32 E_CB_MAP_RELOAD_TIMEOUT    = 2204101   # 对齐 / 替换已成功但重载地图超时（等级 1，数据一致，仅需重载）
#    地图替换（~/replace_map）。replace_map_tool 同为 CLI，由 command_board 按其结果行的 reason 翻译。
int32 E_CB_REPLACE_MAP_NOT_FOUND    = 2204110   # target 或 source 地图不存在 / 未加载任何地图（等级 0）
int32 E_CB_REPLACE_ARG_INVALID      = 2204120   # 入参非法：source 为空、与 target 相同（等级 0）
int32 E_CB_REPLACE_MATCH_FAILED     = 2204130   # dry_run 求变换失败：重叠不足 / 匹配分数不够（等级 0，零文件改动）
int32 E_CB_REPLACE_MATCH_DEGRADED   = 2204140   # 求出变换但未收敛，且未设 accept_degraded（等级 0，零文件改动）
int32 E_CB_REPLACE_TOOL_FAILED      = 2204152   # replace_map_tool 正式执行失败，已回滚 target 底图（等级 2）
int32 E_CB_REPLACE_ROLLBACK_FAILED  = 2204163   # 执行失败且回滚失败：target 底图可能不完整，需人工从 slam/backups/ 恢复（等级 3）
int32 E_CB_REPLACE_SOURCE_TRASH_FAILED = 2204171 # 替换已成功，但 source 移入回收站失败（等级 1，可手动处理）
#    传感器监测（测试用，后期需要统一规划）。
int32 E_CB_NAV_SENSOR_FAULT = 2204182   # 导航必要传感器失效：已取消导航，传感器恢复并稳定后自动解除（等级 2）
