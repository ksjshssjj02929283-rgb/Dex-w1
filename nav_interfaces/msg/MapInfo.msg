# 地图目录条目（get_maps 返回）。信息来自 <map_folder>/<name>/slam/map.yaml，不加载地图也可获取。
string name
string map_path                 # <map_folder>/<name>/slam，与 MappingState.map_path 同口径
float32 total_area_m2           # 图幅总面积；yaml 无此键时为 0
float32 effective_area_m2       # 有效可通行面积；yaml 无此键时为 0
builtin_interfaces/Time updated_at   # map.yaml 最后修改时间
string created_at               # 建图完成（保存 map.yaml）时刻，本地时间 ISO 8601 无时区后缀，如 2026-09-14T16:30:00；来自 map.yaml 的 created_at，旧地图无此键为空
string thumbnail                # 缩略图路径（当前为 <map_path>/map.png，随文件通道方案再定）
