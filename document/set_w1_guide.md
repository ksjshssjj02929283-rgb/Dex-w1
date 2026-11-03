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
| `tool` | 切换 URDF 载荷或编辑质量、质心；URDF 路径按机器人版本自动解析，修改后需 `./set_w1.sh restart` 重启服务 | `./set_w1.sh tool` |

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
