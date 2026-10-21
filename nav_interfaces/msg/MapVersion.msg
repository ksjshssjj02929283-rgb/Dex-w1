# 一张地图的一个版本。底图版本随 get_maps 的 MapInfo.versions 返回；资产版本随 get_assets 的 versions 返回
# （<map>/assets/backups/map_assets.<version_ts>.json，单文件，complete 恒为 true）。
# 底图版本：版本是 <map>/slam/backups/backup.<version_ts>/
# 下的 map.pbstream / map.png / map.yaml 三件套（另含存在时的点云、线特征等附带文件），
# 由定位侧保存、底图编辑、回滚、对齐、替换、重设原点在改动前自动存下——所有写入方同一格式、同一序列。
# 字段只取读目录即可得到的信息，保证 get_maps 列表查询轻量。
string version_ts                     # 目录名 backup.<version_ts> 的后缀，回滚时原样传回
builtin_interfaces/Time saved_at      # 版本产生时刻（由 version_ts 解析，失败取目录修改时间）
string source                         # 版本是哪个操作在改动前存下的：localization_save（定位侧保存）/ grid_edit（底图编辑）/
                                      # rollback（版本回滚）/ align（对齐）/ replace（替换）/ reset_origin（重设原点）/ unknown（未标记的早期版本）
bool complete                         # 三件套是否齐全；不齐全的版本不能回滚
