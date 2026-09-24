std_msgs/Header header   # 状态生效时刻（重定位耗时等指标依赖它）
int32 loc_mode  # 定位模式（传感器来源，与 Relocalize.mode 无关）
int32 status    # 定位状态
float32 score   # 定位置信度（归一化到 [0,1]，定义见下方「score 归一化」段）
int32 error_code  # 最近一次异步重定位（/initialpose topic）的结果；0=正常。service 路径的错误在其响应里返回
uint8 match_level # 匹配质量分档（见下方 MATCH_* 常量）。分档取自一致性检查 + 漂移门控的判定，不是拿 score 比阈值
uint8 score_level # score 的百分比分档（见下方 SCORE_LEVEL_* / SCORE_THRESHOLD_* 常量）


int32 DEFULT_LOC_MODE = 0
int32 LASER_LOC_MODE = 1
int32 RTK_LOC_MODE = 2

int32 LOC_FAILED = -2         # 定位失败
int32 LOC_INIT_FAILED = -1    # 初始化失败
int32 LOC_NOT_INIT = 0        # 未初始化
int32 LOC_INITIALIZING = 1   # 系统初始化中
int32 LOC_INITIALIZED = 2          # 系统初始化成功
int32 LOC_RELOCALIZING = 3    # 重定位中
int32 LOC_SUCCESS = 4           # 定位成功

# 匹配质量分档。⚠️ score 是「点云落在地图内的格子的平均占用概率」，属环境依赖量
# （空旷区未知格概率拉低、走廊里点全打墙上天然偏高），固定百分比阈值在物理上不成立。
# 分档一律取自已有的两套判定：一致性检查（平移/旋转误差 + confidence 三路信号）
# + 漂移门控（static map 独立锚点再匹配 + 去抖）。前端请用 match_level 显示质量，score 仅作诊断。
uint8 MATCH_UNKNOWN = 0   # 定位未运行 / 未收敛
uint8 MATCH_GOOD = 1      # 一致性通过，无自洽漂移嫌疑
uint8 MATCH_DEGRADED = 2  # 检测到自洽漂移但已在 static map 上校正成功（位姿可用，地图/环境存疑）
uint8 MATCH_POOR = 3      # 一致性未通过，或漂移校正失败导致地图更新冻结（位姿不可信）

# ============ score 归一化定义（契约唯一真值源）============
# score = min(原始地图重合度 x 1.25, 1.0)
#   原始地图重合度 = 点云落在地图内的格子的平均占用概率（ProbabilityGrid::GetProbability 均值）
#   x1.25 是既有校准：重合率 0.8 即视为满分（0.8 重合率实际已是优秀定位）
# 取值 [0,1]，乘 100 即百分比。低于 SCORE_THRESHOLD_BAD 时建议走全局重定位兜底。
#
# ⚠️ score 受环境影响：空旷区点云打在未知格（概率约 0.5）会拉低分数，狭窄走廊里点全打墙上则天然偏高。
#    因此 score 适合表达「同一场景内的质量趋势」与人机显示，跨场景绝对值比较需谨慎。
#    「定位能不能用」的权威结论以 match_level 为准（它取自一致性检查 + 漂移门控，不受此环境偏置影响）。
#
# 门限为我方契约定义，消费端（command_board 等）以此为默认值，不应各自另定阈值。
float32 SCORE_THRESHOLD_GOOD = 0.8  # >= 0.8（80%）→ GOOD
float32 SCORE_THRESHOLD_BAD = 0.5   # <  0.5（50%）→ BAD

uint8 SCORE_LEVEL_BAD = 0   # score < 50%：定位质量差，建议全局重定位兜底
uint8 SCORE_LEVEL_FAIR = 1  # 50% <= score < 80%：可用但需持续观察
uint8 SCORE_LEVEL_GOOD = 2  # score >= 80%：定位质量良好

# 定位域状态码（整机状态码体系）：模块 21 dexe_mobile_slam + 子模块 02 localization + 状态序号 2 位 + 等级末位，共 7 位。
# 等级：0 状态反馈 / 1 一级故障 / 2 二级故障 / 3 三级故障，可由 level = code % 10 取出，无需额外字段。
int32 E_NONE = 0
int32 E_LOC_NOT_RUNNING    = 2102010   # 定位未启动，无法重定位（等级 0）
int32 E_LOC_MODE_INVALID   = 2102020   # mode 取值非法（等级 0）
int32 E_LOC_POSE_REQUIRED  = 2102030   # MODE_LOCAL 但未给出合法 pose（等级 0）
int32 E_LOC_POSE_INVALID   = 2102040   # pose 的 frame_id 非 map 或四元数未归一（等级 0）
int32 E_LOC_NO_MAP         = 2102050   # 未加载地图（等级 0）
int32 E_LOC_BUSY           = 2102060   # 上一次重定位仍在进行（等级 0）
int32 E_LOC_TRIGGER_FAILED = 2102072   # 触发失败（引擎拒绝，详见节点日志）（等级 2）
