string id
nav_interfaces/RouteEndpoint from_ep
nav_interfaces/RouteEndpoint to_ep
string   edge_type       # "LINE" | "BEZIER"
bool     bidirectional
float32  max_speed
float32  xy_tolerance
float32  th_tolerance
float32  lane_width
string   obs_policy      # "STOP" | "DETOUR"
geometry_msgs/Point[] control_points

# 平移朝向 rad：全向车以平移方式移动时保持的朝向；NaN=差速方式移动
float32  hold_direction
# 倒车：差速移动过程中车头反向（vx<0）
bool     backward
# 车体四周膨胀大小 m（停障距离检查）；NaN=系统默认
float32  padding
