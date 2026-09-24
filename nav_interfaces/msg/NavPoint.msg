std_msgs/Header header

int32 id          # [deprecated] 仅 RViz marker 编号使用，不是主键；主键为 name，跨节点引用（RouteEndpoint）亦按 name
string name
string type  # 点位类型: CHARGING, STATION, DOORWAIT, PARKING, FREE_GOAL, LIFT

float32 x
float32 y
float32 theta
float32 tolerance  # 导航到达精度（xy方向），0表示使用默认值
