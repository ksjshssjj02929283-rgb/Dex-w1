# 建图状态（结构化，替代裸 std_msgs/Int16 mapping_status）
std_msgs/Header header   # 状态生效时刻
int8    status           # 建图状态，取值见下方常量
string  map_name         # 当前地图名（未设置为空）
string  map_path         # <map_folder>/<map_name>/slam，start_mapping 成功即填写；cancel 后清空
float32 progress         # 0~100：MAPPING 恒 0；SAVING 期间按 node 处理进度 0~90；保存成功后 100
string  stage            # SAVING 期间的阶段名（FINISHING_TRAJECTORIES/OPTIMIZING/SERIALIZING/WRITING_GRID/DONE），其余为空
int32   error_code       # 0 = 无错误；上次操作失败时携带，下次 start_mapping 清零

int8 ERROR = -1    # 建图异常
int8 IDLE = 0      # 空闲（未建图）
int8 MAPPING = 1   # 建图中
int8 SAVING = 2    # 正在保存/收尾建图（finishing+saving）

# 建图域状态码（整机状态码体系）：模块 21 dexe_mobile_slam + 子模块 01 mapping + 状态序号 2 位 + 等级末位，共 7 位。
# 等级：0 状态反馈 / 1 一级故障 / 2 二级故障 / 3 三级故障，可由 level = code % 10 取出，无需额外字段。
int32 E_NONE = 0
int32 E_MAP_ALREADY_RUNNING  = 2101010   # start 时已在建图（等级 0）
int32 E_MAP_NOT_RUNNING      = 2101020   # finish/cancel 时未建图（幂等，success 仍为 true）（等级 0）
int32 E_MAP_NAME_INVALID     = 2101030   # 地图名不合法（等级 0）
int32 E_MAP_NAME_EXISTS      = 2101040   # 重名且未指定 overwrite（等级 0）
int32 E_MAP_FOLDER_MISSING   = 2101052   # map_folder 未配置或不可创建（等级 2）
int32 E_MAP_DIR_RESET_FAILED = 2101062   # 地图目录重建失败（等级 2）
int32 E_MAP_SAVE_FAILED      = 2101072   # 保存任一产物失败（可重试）（等级 2）
int32 E_MAP_ROBOT_MOVING     = 2101080   # 由门面层前置自检产生：机器人非静止（等级 0）
int32 E_MAP_SENSOR_ABNORMAL  = 2101092   # 由门面层前置自检产生：传感器异常（等级 2）
int32 E_MAP_BATTERY_LOW      = 2101101   # 由门面层前置自检产生：电量不足（等级 1）
int32 E_MAP_SAVING           = 2101110   # 收尾保存进行中，拒绝 start/cancel（等级 0）
int32 W_MAP_AREA_LIMIT       = 2101501   # 有效面积达上限（warning，不中断）（等级 1）
