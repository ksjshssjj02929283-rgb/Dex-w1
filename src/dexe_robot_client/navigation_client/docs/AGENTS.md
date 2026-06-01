# navigation_client 文档协作规则

## 适用范围

本文件适用于 `src/dexe_robot_client/navigation_client/docs/` 目录，以及 `navigation_client` 相关接口说明、问题清单和维护记录。

## 文档语言

- 默认使用中文编写章节标题、说明和示例描述。
- Python 类名、函数名、ROS topic、service、action、参数名和状态字符串保留原文。

## 同步要求

- 修改 `src/dexe_robot_client/navigation_client/` 下的接口、状态语义、ROS topic、service、action、参数或仿真/真实模式初始化逻辑时，应同步更新本目录 `README.md`。
- 修改 `W1Manager` 对 navigation client 的包装接口时，应同步检查本文档中的“上层包装接口”章节。
- 新增或移除 ROS 依赖时，应同步维护 ROS 接口表，必要时同步仓库根目录 `docs/README.md`。
- 记录问题时应区分“已确认代码现状”和“推断风险”，避免把待验证假设写成结论。

## 构建与验证

- 除非用户明确要求，不主动执行 `colcon build`、`catkin build`、`cmake --build`、`make` 等构建命令。
- 文档类变更优先用 `rg`、`sed`、`git diff` 等静态方式核对，不需要启动 ROS 节点。
