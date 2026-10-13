# dexe_robot_server 介绍

> **项目分为两个部分：通用库部分 `robot_server_lib`，和仿真部署实现 `sim_server`。同时 `sim_server` 也是一个模板，可以参考它的实现快速部署到其他机器人上。**

## 新架构需使用分支dev_w1

## 依赖库安装

```bash
# ROS2 公钥导入 如果已有公钥可跳过
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc > /dev/null

# 安装依赖库
sudo apt update
sudo apt install nlohmann-json3-dev ros-humble-ruckig ros-humble-pinocchio

# 底盘相关依赖库
sudo apt install ros-humble-nav-msgs ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

# 代码格式(可选)
sudo apt install clang-format

# 可视化(可选)
pip install meshcat
```

> **注意**：如已通过源码编译安装，无需再安装 ROS 版本，否则会链接冲突。

> **安装问题排查**：如遇包找不到，请检查 `/etc/apt/source.list.d/ros2.list`，源需为：
> `deb [arch=arm64 trusted=yes] http://packages.ros.org/ros2/ubuntu jammy main`

## 编译项目

```bash
./dexe_robot_server/build.sh
```

## 运行项目

运行部署的包, 运行以下可开启仿真及可视化

```bash
./dexe_robot_server/run_sim.sh
# 仿真下如果需要运行类似slowly_move_to.py脚本需手动把真机话题都加上_sim后缀修改为仿真的, 如/feedback/joint -> /feedback/joint_sim
```

如运行真机w1 server则需运行
```bash
./dexe_robot_server/run_w1.sh
```

运行测试
```bash
./dexe_robot_server/test.sh
```

## 以下为可配置项, 非特殊需要可以保持默认
### 编译可配置项

**可视化仿真：**

默认开启, 如需关闭则

```bash
colcon build --cmake-args -DENABLE_SIM_SERVER=OFF
```

**力控仿真模式：需在可视化仿真开启后开启，运行力控仿真，输出的关节位置为力控仿真后的关节位置**

默认关闭，如需开启则
```bash
colcon build --cmake-args -DENABLE_TORQUE_CONTROL_SIMULATION=ON
```

---

## 配置参数

项目使用 YAML 配置，不再使用旧版 `robot.cfg`：

| 配置 | 路径 | 用途 |
|---|---|---|
| 随包默认配置 | [`w1_server/robot_config.yaml`](w1_server/robot_config.yaml) | 提供完整字段、默认参数和 `config_version` |
| 真机用户配置 | `~/dexe_robot_server_data/robot_config.yaml` | `w1_server` 实际加载并保留本机参数的配置 |

首次启动 `w1_server` 时，如果用户配置不存在，程序会从随包默认配置创建一份。后续启动会比较 `config_version`：默认配置版本更新时替换旧用户配置；版本一致时补齐缺失字段和关节属性，并保留已有参数。`~/.cache/dexe_data/config.yaml` 中的硬件信息也会在启动时同步到机器人配置。

`sim_server` 直接读取随包默认配置，不读写真机用户配置，避免仿真和真机进程并发修改同一文件。

配置以 `robot_type` 和 `version` 选择对应的型号配置块。例如：

```yaml
config_version: v05
robot_type: w1
version: v024

file_path:
  robot_urdf: ../install/robot_server_lib/share/robot_server_lib/robot_models/w1/robot.urdf

joints_primary_names: [LEFT_J1, LEFT_J2, LEFT_J3, LEFT_J4, LEFT_J5, LEFT_J6, LEFT_J7]
joints_secondary_names: [RIGHT_J1, RIGHT_J2, RIGHT_J3, RIGHT_J4, RIGHT_J5, RIGHT_J6, RIGHT_J7]

w1_v02:
  enable_streaming_ruckig: true
  joint_streaming_bounds:
    max_velocity: 3.1416
    max_acceleration: 9.4248
    max_jerk: 157.08
  safety_check_config:
    enable_position_limit_check: true
    enable_torque_limit_check: true
    enable_velocity_limit_check: true
```

主要配置内容包括机器人型号与 URDF、关节映射和运动分组、安全限制、PD 与柔顺控制、轨迹滤波与抑振、Streaming 限制、动力学参数、工具补偿以及底盘和 IMU 参数。

### Drag 模式附加阻尼

Drag 的输出力矩为 `tau_drag = tau_ff - D1*dq - D2*abs(dq)*dq + tau_dead`，其中当 `abs(dq) < dq_deadzone` 时 `tau_dead = F_dead*sgn(dq)`，否则为零；等于零速度时 `sgn(0)=0`。`dq` 使用反馈速度（rad/s），阈值由所有关节共用。`tau_ff` 仍为原逆动力学前馈，包含原有的重力、科氏力、摩擦和已启用的工具补偿；附加阻尼和死区补偿不替换辨识摩擦。仅 Drag 使用，其他控制模式不变。

在实际使用的 `w1_v02` / `w1_v03` 配置段分别设置左右臂的 1×7 系数，按各臂 J1～J7 排列：

```yaml
  drag_damping_d1_primary: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  drag_damping_d2_primary: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  drag_damping_d1_secondary: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  drag_damping_d2_secondary: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  drag_dq_deadzone: 0.1
  drag_dead_compensation_primary: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  drag_dead_compensation_secondary: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

`primary` 是左臂，`secondary` 是右臂，数组按 J1～J7 排列。D1 单位为 N·m·s/rad，D2 单位为 N·m·s²/rad²，`F_dead` 单位为 N·m；数组必须恰好含 7 个有限非负数。死区阈值 `drag_dq_deadzone` 默认 0.1 rad/s，需为有限非负数，整机所有关节共用。`F_dead` 默认全零，保持原 Drag 行为；非手臂关节不使用死区补偿。升级需要重新编译安装库和服务端；之后修改 YAML 系数只需重启服务，不需要重新辨识动力学参数。建议从零开始小幅调节，尤其二次阻尼随速度平方增大。

可通过以下入口修改配置：

```bash
./set_w1.sh config          # 打开真机用户配置
./set_w1.sh ruckig true     # 修改 YAML 并同步运行时参数
./set_w1.sh filter true     # 修改滤波配置并同步运行时参数
./set_w1.sh zvd true        # 修改 Input Shaper 配置并同步运行时参数
./set_w1.sh tool            # 编辑末端工具动力学配置
```

通过 `/control/set_property` 修改的参数通常只在当前进程中生效；需要重启后保留的参数应写入用户 YAML。完整配置读写逻辑参见 [dexe_robot_server 设计文档](document/dexe_robot_server_design.md#5-配置参数)，脚本命令参见 [set_w1.sh 使用指南](document/set_w1_guide.md)。

---

## 代码风格检查

开发者提交代码前需统一代码风格, 请运行：

```bash
./dexe_robot_server/check_style.sh
```
