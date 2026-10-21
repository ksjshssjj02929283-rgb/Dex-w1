# 地图状态原子下发消息（latched: transient_local, depth 1, reliable）
# folder + name + epoch + stamp 一条消息原子下发，消除多 topic 时序竞争。
# epoch：单调递增版本号，任何一次地图加载（含同名重载 / 回退）都 +1；
#        消费方比对 epoch（而非 name）判断是否需要重载，从而不漏检同名内容变更。
#        0 表示尚未加载任何地图。不持久化，节点重启从 0 重新计数。
std_msgs/Header header
string map_folder
string map_name
uint64 epoch

# 地图管理域状态码（整机状态码体系）：模块 22 dexe_mobile_nav + 子模块 02 map_manager + 状态序号 2 位 + 等级末位，共 7 位。
# 等级：0 状态反馈 / 1 一级故障 / 2 二级故障 / 3 三级故障，可由 level = code % 10 取出，无需额外字段。
int32 E_NONE = 0
int32 E_MAP_OP_INVALID     = 2202010   # MapManagerCommand.op 未填或非法（等级 0）
int32 E_MAP_NAME_INVALID   = 2202020   # 地图名为空（等级 0）
int32 E_MAP_NOT_FOUND      = 2202030   # 地图不存在（等级 0）
int32 E_MAP_IN_USE         = 2202040   # 试图删除当前已加载的地图（等级 0）
int32 E_MAP_NAME_EXISTS    = 2202050   # rename 目标名已存在（等级 0）
int32 E_MAP_FOLDER_MISSING = 2202062   # map_folder 未配置（等级 2）
int32 E_MAP_LOAD_FAILED    = 2202072   # yaml/png 读取或旧布局迁移失败（等级 2）
int32 E_MAP_IO_FAILED      = 2202082   # 删除/重命名时文件系统错误（等级 2）
int32 E_MAP_TRASH_DISABLED = 2202090   # 回收站未启用（map_trash_enable=false），无归档可查询/恢复（等级 0）
int32 E_MAP_EDIT_INVALID   = 2202100   # 底图编辑 / 显示朝向参数非法：工具、形状、点数、宽度、角度非有限值（等级 0）
int32 E_MAP_GRID_REVISION_MISMATCH = 2202110   # 底图已被其它写入方改过（编辑、回滚、替换、对齐），需重新读取（等级 0）
int32 E_MAP_VERSION_NOT_FOUND = 2202120   # 指定的底图版本不存在或三件套不齐，无法回滚（等级 0）
int32 E_MAP_NO_SPACE         = 2202132   # 地图导出 / 导入时磁盘空间不足（等级 2）
int32 E_MAP_IMPORT_FILE_INVALID = 2202140   # 导入文件不可用：不在导入目录、不存在、不是普通文件、超过大小上限、不是 zip（等级 0）
int32 E_MAP_IMPORT_INCOMPATIBLE = 2202150   # 导入文件不兼容：缺清单、格式版本不支持、路径不安全、校验和不符、缺必需文件、数据版本不支持（等级 0）
