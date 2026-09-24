# 导航域状态（/navigation/state，latched depth 1；状态变化即发 + 1 Hz 心跳）
# 一条话题回答"导航怎么样了"：当前任务 / 状态 / 错误码 / 目标 / 进度 / 路径版本。
std_msgs/Header header
string task_id                     # 门面受理时生成，贯穿 state / global_path / action result；空 = 从未有任务
uint8  state
int32  error_code                  # 0 = 正常；FAILED / CANCELLED 时填 13xx
uint8  target_type
string target_name                 # TARGET_POINT：点名；TARGET_ROUTE：路线名；TARGET_POSE：空
geometry_msgs/PoseStamped   target_pose   # 最终目标位姿（map 系）
geometry_msgs/PoseStamped[] waypoints     # 途经目标序列（点位 / 位姿任务长度 1；路线任务 = 各点位姿）
uint32  current_index              # 正在前往 waypoints 的第几个（0-based）
float32 distance_remaining         # m，来自 nav2 feedback
float32 eta_sec                    # s，来自 nav2 feedback
uint32  poses_remaining            # 路线任务剩余点数（nav2 NavigateThroughPoses feedback）
uint32  recoveries                 # nav2 恢复行为次数
uint64  plan_seq                   # 全局路径版本；与 global_path.plan_seq 对应，0 = 尚无路径
string  message                    # 人读附注（失败原因 / 阻塞原因）
float32 speed_limit                # 当前限速（nav_command SET_SPEED_LIMIT 设定，跨任务保持）；0 = 无限速
bool    speed_percentage           # true：speed_limit 为最大速度百分比；false：m/s

# 任务状态机
uint8 IDLE      = 0    # 无任务（启动后或上一任务终态发布之后仍保留终态，直到新任务受理）
uint8 PENDING   = 1    # 已受理，等待 nav2 接受
uint8 RUNNING   = 2    # nav2 执行中
uint8 PAUSED    = 3    # nav_command PAUSE：nav2 goal 已取消、目标保留，RESUME 重发
uint8 BLOCKED   = 4    # 停障等待超过阈值（error_code=0）/ 急停（error_code=2201123）；解除后自动回 RUNNING
uint8 SUCCEEDED = 5
uint8 FAILED    = 6
uint8 CANCELLED = 7

# 目标类型
uint8 TARGET_NONE  = 0
uint8 TARGET_POINT = 1
uint8 TARGET_POSE  = 2
uint8 TARGET_ROUTE = 3

# 导航域状态码（整机状态码体系）：模块 22 dexe_mobile_nav + 子模块 01 navigation + 状态序号 2 位 + 等级末位，共 7 位。
# 等级：0 状态反馈 / 1 一级故障 / 2 二级故障 / 3 三级故障，可由 level = code % 10 取出，无需额外字段。
int32 E_NONE = 0
int32 E_NAV_TARGET_NOT_FOUND  = 2201010   # 点名 / 路线名不存在（等级 0）
int32 E_NAV_GOAL_UNREACHABLE  = 2201020   # 规划失败：未产生任何全局路径（目标在障碍内 / 无路可达）（等级 0）
int32 E_NAV_PATH_BLOCKED      = 2201031   # 执行中因障碍失败（停障等待超时 / 控制器碰撞退出）（等级 1）
int32 E_NAV_SENSOR_LOST       = 2201042   # 定位未就绪或丢失（LocalizationState 非 LOC_SUCCESS）（等级 2）
int32 E_NAV_NOT_READY         = 2201052   # nav2 未就绪 / 无地图（等级 2）
int32 E_NAV_BUSY              = 2201060   # 已有任务且不允许抢占（等级 0）
int32 E_NAV_CANCELLED         = 2201070   # 被取消（nav_command CANCEL / 新任务抢占 / 客户端取消）（等级 0）
int32 E_NAV_CONTROLLER_FAILED = 2201082   # nav2 执行失败（其它原因，message 附 nav2 文本）（等级 2）
int32 E_NAV_TIMEOUT           = 2201091   # 进度看门狗超时（等级 1）
int32 E_NAV_OP_INVALID        = 2201100   # nav_command.op 非法 / 参数非法 / 当前状态不允许（等级 0）
int32 E_NAV_NOT_SUPPORTED     = 2201110   # 预留接口位（CHECK_REACHABLE）尚未实现（等级 0）
int32 E_NAV_ESTOP             = 2201123   # 急停中：BLOCKED 的阻塞原因 / 急停中受理新目标或 RESUME 的失败码（等级 3）
