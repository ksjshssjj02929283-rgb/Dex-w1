# 当前任务的全局路径（/navigation/global_path，latched depth 1）
# 每次规划 / 重规划发一次；任务结束发一条 poses 为空的消息。map 系。
std_msgs/Header header
string task_id
uint64 plan_seq                                 # 单调递增，与 NavigationState.plan_seq 对应
nav_msgs/Path path                              # 几何路径（map 系）
nav_interfaces/RoutePlanSegment[] segments      # 路网模式附带的段语义（限速 / 车道子类 / 楼层）；非路网模式为空
