# 单个轨迹点 —— 携带来自 RoutePlanSegment 的段语义
geometry_msgs/PoseStamped  pose
float32                    speed_hint    # 期望速度上限 (m/s)；0 = 系统默认（不限速）
string                     region_type   # 来自 RoutePlanSegment.lane_subtype（corridor/elevator/transition/...）
string                     segment_id    # 来自 RoutePlanSegment.edge_id（可追溯到段）
