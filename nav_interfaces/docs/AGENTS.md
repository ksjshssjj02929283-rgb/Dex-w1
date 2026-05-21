# nav_interfaces/docs 维护规则

## 文档范围

- 本目录记录 `nav_interfaces` 中 msg、srv、action 接口的补充说明和维护注意事项。
- 修改接口文件、生成列表或接口语义时，应同步更新本目录文档。

## 当前接口补充

- `PreciseControl.action` 是 Action 版底盘末端精控接口，Goal 使用 `geometry_msgs/PoseStamped target_pose`。
- `PreciseControl.action` 已加入 `CMakeLists.txt` 的 `ACTION_FILES`，新增或重命名 action 时需要同步维护该列表。
