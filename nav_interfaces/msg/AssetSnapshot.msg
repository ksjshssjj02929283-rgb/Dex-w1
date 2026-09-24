# 地图资产全量快照 + 管理器状态（~/assets，latched depth 1）。
# 任何一次资产变更（含 RViz 侧编辑）、地图切换、set_assets / save 结果都推送一次；晚订阅者拿到最新一版。
# 六类资产与 GetAssets 响应一致；X-Wiz 按 map_epoch + 内容刷新页面，按 error_code 提示最近一次写入结果。
std_msgs/Header header
string map_name
uint64 map_epoch
bool   loaded              # 已跟随 map_state 切到某张图并加载其资产；false 时六类数组为空
int32  error_code          # 最近一次 set_assets / save 的结果；0 = 正常（各类数量 = 对应数组长度）
nav_interfaces/NavPoint[]      points
nav_interfaces/AreaInfo[]      areas
nav_interfaces/RoomInfo[]      rooms
nav_interfaces/Route[]         routes       # 命名路线（点位序列）
nav_interfaces/RouteJunction[] junctions    # 路网拓扑
nav_interfaces/RouteEdge[]     edges

# 地图资产域状态码（整机状态码体系）：模块 22 dexe_mobile_nav + 子模块 03 map_asset_manager + 状态序号 2 位 + 等级末位，共 7 位。
# 等级：0 状态反馈 / 1 一级故障 / 2 二级故障 / 3 三级故障，可由 level = code % 10 取出，无需额外字段。
int32 E_NONE = 0
int32 E_ASSET_MODE_INVALID   = 2203010   # SetAssets.mode / Set*.mode 非法（等级 0）
int32 E_ASSET_EPOCH_MISMATCH = 2203020   # expected_epoch 与当前地图 epoch 不符（编辑期间地图被切换）（等级 0）
int32 E_ASSET_NO_MAP         = 2203030   # 尚未加载任何地图（等级 0）
int32 E_ASSET_VALIDATION     = 2203040   # 字段校验失败，见 rejected[]（等级 0）
int32 E_ASSET_REF_UNRESOLVED = 2203050   # 引用的点位/节点在应用后的集合中不存在，见 rejected[]（等级 0）
int32 E_ASSET_SAVE_FAILED    = 2203062   # 内存已应用，落盘失败（可重试 save）（等级 2）
int32 E_ASSET_NOT_FOUND      = 2203070   # 单条接口：目标不存在（UPDATE_ONLY / REMOVE）（等级 0）
int32 E_ASSET_EXISTS         = 2203080   # 单条接口：ADD_ONLY 时目标已存在（等级 0）
