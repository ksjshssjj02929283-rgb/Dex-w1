# set_w1.sh 使用指南

`set_w1.sh` 是 dexe_robot_server 的一站式运维脚本，集成了机器人控制、状态监控、调试工具、参数配置等常用功能。

## 1. 如何打开

在终端中进入项目目录并执行：

```bash
cd ~/w1/dexe_robot_server
./set_w1.sh <设置指令>  #下方有各指令示例
```

## 2. 命令分类

### 2.1 状态监控

| 命令 | 说明 | 示例 |
|------|------|------|
| `echo` | 持续监听机器人状态 topic | `./set_w1.sh echo` |
| `echo_once [field]` | 单次查询机器人状态，可按字段名过滤 | `./set_w1.sh echo_once server_version` |
| `echo_sim` | 持续监听仿真状态 topic | `./set_w1.sh echo_sim` |
| `echo_sim_once [field]` | 单次查询仿真状态 | `./set_w1.sh echo_sim_once` |
| `state` | 监控反馈状态（终端仪表盘） | `./set_w1.sh state` |
| `shm` | 监控共享内存数据 | `./set_w1.sh shm` |
| `watch` | 监控 EtherCAT 从站状态 | `./set_w1.sh watch` |
| `log [lines]` | 实时跟踪机器人日志 | `./set_w1.sh log 100` |

### 2.2 机器人运动

| 命令 | 说明 | 示例 |
|------|------|------|
| `home` | 全身回到零位 | `./set_w1.sh home` |
| `standby` | 全身回到待机位 | `./set_w1.sh standby` |
| `left` | 左臂移动（7 关节，可写 home/standby 或自定义数组） | `./set_w1.sh left '[0.0, -1.4, 0.0, -0.6, 0.0, -0.5, -0.5]'` |
| `right` | 右臂移动 | `./set_w1.sh right standby` |
| `stop_move` | 停止当前运动 | `./set_w1.sh stop_move` |

### 2.3 系统服务

| 命令 | 说明 | 示例 |
|------|------|------|
| `start` | 启动 dexe-system 服务并监控 EtherCAT | `./set_w1.sh start` |
| `stop` | 停止 dexe-system 服务 | `./set_w1.sh stop` |
| `restart` | 重启 dexe-system 服务并监控 EtherCAT | `./set_w1.sh restart` |

*注：start/stop/restart 需要 sudo 权限，脚本会尝试使用默认密码自动提权。*

### 2.4 机器人参数与配置

| 命令 | 说明 | 示例 |
|------|------|------|
| `drag 0/1` | 启用/禁用拖拽模式 | `./set_w1.sh drag 1` |
| `clear` | 清除错误 | `./set_w1.sh clear` |
| `sleep` | 进入休眠模式 | `./set_w1.sh sleep` |
| `quickstop` | 急停 | `./set_w1.sh quickstop` |
| `pos_limit 0/1` | 启用/禁用位置限位检查 | `./set_w1.sh pos_limit 0` |
| `mode <n>` | 切换运行模式 | `./set_w1.sh mode 3` |
| `safe_level <0.1-1.0>` | 设置安全等级(越高最大输出力矩越大) | `./set_w1.sh safe_level 0.3` |
| `ruckig true/false` | Ruckig 轨迹插值开关 | `./set_w1.sh ruckig true` |
| `filter true/false` | 巴特沃斯滤波器开关 | `./set_w1.sh filter true` |
| `zvd true/false` | 输入整形器开关 | `./set_w1.sh zvd true` |
| `config` | 编辑 robot_config.yaml 文件 | `./set_w1.sh config` |
| `hardware` | 查看 hardware.yaml | `./set_w1.sh hardware` |
| `tool` | 末端工具重力补偿参数(修改后需./set_w1.sh restart重启服务) | `./set_w1.sh tool` |

#### 前端启动前修改工具配置

推荐前端直接调用 Python 脚本，不经过 `set_w1.sh` 的服务环境和 sudo 初始化。
依赖 Python 3 和 PyYAML（`yaml` 模块），默认文件为当前用户的
`~/dexe_robot_server_data/robot_config.yaml`，可用 `--config /absolute/path/robot_config.yaml` 指定。

```bash
# 左手：按名称修改质量和质心，即使 enabled=false 也可以修改
python3 script/set_tool_dynamics.py --arm primary --tool BrainCo_Revo1_R --mass-kg 0.65 --com-m 0.01 0 0.08

# 右手：修改 Default，并同时开启该侧补偿
python3 script/set_tool_dynamics.py --arm secondary --tool Default --mass-kg 0.2 --com-m 0 0 0.03 --enabled true

# 仅关闭左手补偿，不改质量、质心
python3 script/set_tool_dynamics.py --arm primary --enabled false

# 仅修改某个已有工具的质量；也可以只传 --com-m X Y Z
python3 script/set_tool_dynamics.py --arm secondary --tool Linker_L6 --mass-kg 0.5

# 查看文件配置
python3 script/set_tool_dynamics.py --list
```

参数模式规则：

- `--arm` 必填，`primary` 为左手，`secondary` 为右手；每次调用修改一侧的一个工具。
- `--enabled` 接受 `true` / `false`。未传入时保留原值；编辑质量、质心不要求已开启补偿。
- 修改质量或质心必须提供 `--tool`，按本侧已有名称精确匹配，区分大小写，默认项名称为 `Default`。
  未找到时报错，不新增工具，也不回退到 Default；不修改 `left_hand_type` / `right_hand_type`。
- 质量必须是有限非负数（kg）；质心必须是三个有限数值（m，EE/法兰坐标系），可以为负。
  零质量合法。修改后该工具的完整质量、质心必须都合法，否则整次不保存。
- 目标字段须已存在，质量/开关须是单行标量，质心须是三标量序列。支持行内列表和多行列表。
  目标路径重复键、YAML 合并键、文档中的锚点/别名会报错，需要先整理为独立、明确的配置。
- 单次调用全部校验通过后才保存；只替换指定字段的数值，保留其他配置、注释和排版。
  使用同目录临时文件原子替换，并通过锁文件串行处理本脚本的参数模式及交互模式写入。
  外部编辑器不参与该锁，避免同时修改同一文件。
- 成功退出码为 `0`，stdout 输出一行 JSON，包含 `ok/config/arm/tool/updated`；
  参数或配置校验失败返回 `2`，文件访问/写入失败返回 `1`，错误信息写到 stderr。
  参数模式无交互提示、不自动启动或重启服务。前端收到成功后再启动 w1_server；已运行的服务需重启。
- 不带修改参数时进入终端交互模式，`--list` 仅显示配置。交互模式同样只替换目标数值，
  保留其他配置、注释、缩进和列表排版；删除工具只移除该工具条目及其内部注释。
  每次保存都会重新读取文件，不会用启动时的整份旧配置覆盖外部已经保存的其他修改。
  保存失败会显示错误，保留原文件及菜单中的原值。


### 2.5 GUI 工具

| 命令 | 说明 | 示例 |
|------|------|------|
| `ctrl` | 关节控制界面（键盘方向键调节） | `./set_w1.sh ctrl` |
| `dyn` | 动力学参数辨识 GUI | `./set_w1.sh dyn` |
| `gui` | 机器人动作演示启动器 | `./set_w1.sh gui` |

### 2.6 动力学参数相关

| 命令 | 说明 | 示例 |
|------|------|------|
| `codegen` | 生成动力学符号 C++ 代码 | `./set_w1.sh codegen` |
| `dh [left/right]` | 计算 DH 参数 | `./set_w1.sh dh left` |
| `trajgen` | 生成激励轨迹（用于参数辨识） | `./set_w1.sh trajgen` |
| `vis` | 启动 Meshcat 可视化节点 | `./set_w1.sh vis` |
