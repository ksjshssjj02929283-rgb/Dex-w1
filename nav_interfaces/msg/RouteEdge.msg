string id
nav_interfaces/RouteEndpoint from_ep
nav_interfaces/RouteEndpoint to_ep
string   edge_type       # "LINE" | "BEZIER"
bool     bidirectional
geometry_msgs/Point[] control_points

# 跟踪方式（高级属性，默认 0=按 schedule_planner 默认逻辑自动选择；
# map_asset_manager 的 rviz 新建路径工具不暴露该字段）
uint8    mode
uint8    MODE_DIFF = 1                 # 差速
uint8    MODE_OMNI_TRANSLATION = 2     # 全向平移（依赖 hold_direction，为 NaN 时 schedule_planner 会降级为 MODE_DIFF）
uint8    MODE_OMNI_TRANS_ROTATION = 3  # 全向一边平移一边旋转

# 倒车：差速移动过程中车头反向（vx<0）
bool     backward
# 平移朝向 rad：全向车以平移方式移动时保持的朝向；NaN=差速方式移动
float32  hold_direction
# 车体四周膨胀大小 m（停障距离检查）；NaN=系统默认
float32  padding
float32  max_speed

# 路网宽度 m：机器人允许通行的范围，规划路径不得超出；0 = 不限定（沿线按机器人外接半径校验）。
# 非 0 时不得小于机器人外接直径，保存时沿线按半宽校验可通行（E_ASSET_GEOMETRY_INVALID / width_too_small）
float32  lane_width
# 暂时不用
float32  xy_tolerance
float32  th_tolerance
string   obs_policy      # "STOP" | "DETOUR"
