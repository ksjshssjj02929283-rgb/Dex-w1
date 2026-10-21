# 地图目录条目（get_maps 返回）。信息来自 <map_folder>/<name>/slam/map.yaml，不加载地图也可获取。
string name
string map_path                 # <map_folder>/<name>/slam，与 MappingState.map_path 同口径
float32 total_area_m2           # 图幅总面积；yaml 无此键时为 0
float32 effective_area_m2       # 有效可通行面积；yaml 无此键时为 0
builtin_interfaces/Time updated_at   # map.yaml 最后修改时间
string created_at               # 建图完成（保存 map.yaml）时刻，本地时间 ISO 8601 无时区后缀，如 2026-09-14T16:30:00；来自 map.yaml 的 created_at，旧地图无此键为空
string data_version             # 同目录 map.pbstream 的数据版本，来自 map.yaml 的 data_version：
                                #   "0.3.0" 等 = 该版本（0.3.0 起带原始 range data，离线对齐 / 替换可无损重建）；
                                #   "legacy" = 0.3.0 之前的旧版 pbstream；空 = 未知（yaml 由旧版本程序写出，无此键）
string thumbnail                # 缩略图路径（当前为 <map_path>/map.png，随文件通道方案再定）
string grid_revision            # 底图内容指纹（map.png 像素 + 分辨率 + 原点），供 map_command OP_EDIT_GRID 的 expected_revision
float32 display_yaw             # 显示朝向（弧度，(-π, π]），来自 map.yaml 的 display_yaw；未设置为 0
builtin_interfaces/Time last_edited_at   # 底图最后编辑时间：map.yaml 的 last_edited_at（建图 / 定位侧保存 / 对齐 / 替换 / 重设原点 /
                                         # 底图编辑 / 显示朝向 / 回滚时写入，加载与读取不动它）；只反映底图，与资产无关
                                         # （资产见 GetAssets.last_edited_at）。旧地图无此键时为 map.png、map.yaml mtime 的较大值
nav_interfaces/MapVersion[] versions     # 底图版本（slam/backups），新的在前；回滚：任意地图 map_command OP_ROLLBACK，当前地图 command_board rollback_map
