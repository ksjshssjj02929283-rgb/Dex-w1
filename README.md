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

# Ros接口库
git clone git@69.235.177.182:dexembodied/dexe_interfaces.git #需要dexe_interfaces main分支, 放在dexe_robot_server同级目录下即可, 无需手动编译

# dexRP机器人规划库
git clone git@69.235.177.182:dexembodied/dexe_robot_planning.git #需要dexe_robot_planning接口包, 需要手动安装
cd dexe_robot_planning
git checkout dev # 切换dev分支
pip install conan  # 安装conan包管理工具
./scripts/build.sh  # 运行编译脚本
cd build && sudo make install # 安装编译好的库

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

**gtest单元测试：**

默认开启, 如需关闭则

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF
```

**力控仿真模式：需在可视化仿真开启后开启，运行力控仿真，输出的关节位置为力控仿真后的关节位置**

默认关闭，如需开启则
```bash
colcon build --cmake-args -DENABLE_TORQUE_CONTROL_SIMULATION=ON
```

---

## 运行参数设置

运行前需在 `/xxx/robot.cfg` 文件中配置参数。 xxx为具体的implement包, 如w1_control, sim_server中等

### 配置参数说明

**robot.cfg** 示例：

```json
{
    "robot_name": "W1Industrial",
    "left_arm_name": "W1LeftArmIndustrial",
    "right_arm_name": "W1RightArmIndustrial",
    "joints_primary": [6, 7, 8, 9, 10, 11, 12],
    "joints_secondary": [13, 14, 15, 16, 17, 18, 19]
}
```

**参数说明：**

- `robot_name`：机器人模型名称，用于在 `/robot_models` 中根据 urdf 建立模型
- `left_arm_name`：左臂控制器名称（单臂机器人可不填）
- `right_arm_name`：右臂控制器名称（单臂机器人可不填）
- `joints_primary`：主要关节（左臂）索引数组（单臂机器人可不填）
- `joints_secondary`：次要关节（右臂）索引数组（单臂机器人可不填）

> **注意**：关节索引从 0 开始计数，请根据实际机器人 URDF 模型中的关节定义配置正确的索引值。

---

## 代码风格检查

开发者提交代码前需统一代码风格, 请运行：

```bash
./dexe_robot_server/style.sh
```