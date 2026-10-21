# 地图回收站条目（get_trashed_maps 返回）。
# 归档时整目录 rename 进回收站，结构完整保留（<entry>/slam/map.yaml 仍在），
# 因此面积 / 建图时刻 / 缩略图等元数据与正常地图同口径可读。
string entry_name                  # 回收站内的目录名，如 office_20260922_143012
string original_name               # 解析出的原地图名，如 office；OP_RESTORE 的 new_name 留空时即恢复成它
builtin_interfaces/Time archived_at  # 归档时刻（解析自 entry_name 的时间戳段）
nav_interfaces/MapInfo map_info    # 复用正常地图的元数据；其中 name = entry_name、map_path 指向回收站内路径
