# 文档协作规则

## 适用范围

本文件适用于 `docs/` 目录及仓库内与文档同步相关的维护工作。

## 文档语言

- 默认使用中文编写说明、章节标题和示例描述。
- 命令、路径、ROS topic、service、action、参数名和代码标识符可保留原文。

## 同步要求

- 修改代码、ROS 接口、参数、launch、配置、运行方式或行为语义时，应同步更新本目录下的 `README.md`。
- 如果改动影响用户入口，也应同步更新仓库根目录 `README.md`。
- 文档应说明实际行为、失败返回、接口语义和关键约束，避免只写泛化模板。
- 后续修改 `src/dexe_robot_client/navigation_client/` 的 topic、service、action、消息类型或 Nav2 参数依赖时，应同步维护 `docs/README.md` 中的 ROS/Nav2 接口依赖清单和最小 apt 安装命令。
- 如果新增依赖会拉起完整 Nav2 栈或可能影响 OMPL 安装方式，应在 `dependencies/README.md` 中说明冲突风险和推荐安装边界。

## 构建约束

- 除非用户明确要求，不主动执行 `colcon build`、`catkin build`、`cmake --build`、`make`、`npm run build` 等构建命令。
- 需要验证时，优先使用语法检查、静态检查或局部脚本；需要构建验证前先询问用户。
