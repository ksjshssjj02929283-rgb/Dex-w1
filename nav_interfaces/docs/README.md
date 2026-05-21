# nav_interfaces 文档

本目录记录 `nav_interfaces` 接口包的补充说明。该包用于生成导航相关 msg、srv 和 action 接口。

## Action 接口

### `PreciseControl`

`PreciseControl` 用于 Action 版底盘末端精控节点。

接口文件：

- `action/PreciseControl.action`

字段：

- Goal：`geometry_msgs/PoseStamped target_pose`
- Result：`bool success`、`string message`
- Feedback：`string current_status`

当前调用方：

- `mobile_perception/mobile_control_action_node`

维护注意：

- 修改 action 字段会影响生成的 C++/Python 接口，调用方需要同步更新并重新构建接口包。
- `target_pose` 当前约定为精控目标位姿，调用方可使用非 `map` frame，具体 TF 转换由执行节点处理。
