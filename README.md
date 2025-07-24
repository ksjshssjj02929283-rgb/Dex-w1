# dexe_robot_server 介绍

> **项目分为两个部分：通用库部分 `robot_server_lib`，和仿真部署实现 `sim_server`。同时 `sim_server` 也是一个模板，可以参考它的实现快速部署到其他机器人上。**

## 依赖库安装

```bash
# ROS2 公钥导入
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc > /dev/null

sudo apt update

# 安装依赖库
sudo apt install nlohmann-json3-dev
sudo apt install ros-humble-ruckig
sudo apt install ros-humble-pinocchio
sudo apt install clang-format

pip install mechcat
```

> **注意**：如已通过源码编译安装，无需再安装 ROS 版本，否则会链接冲突。

### 如果需要ruckig 源码编译安装:

```bash
git clone https://github.com/pantor/ruckig.git
cd ruckig
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

> **安装问题排查**：如遇包找不到，请检查 `/etc/apt/source.list.d/ros2.list`，源需为：  
> `deb [arch=arm64 trusted=yes] http://packages.ros.org/ros2/ubuntu jammy main`

---

## 快速部署

需要dexe_interfaces接口包, source /dexe_interfaces/install/setup.bash

dexe_interfaces gitlab: http://69.235.177.182:8081/dexembodied/dexe_interfaces

如有已部署的包如w1_control也放在同级目录下

---

## 编译项目

```bash
colcon build
```

仿真可视化编译默认开启, 如需关闭则

```bash
colcon build --cmake-args -DENABLE_SIM_SERVER=OFF
```
如果同时真机在运行, 仿真位置会同步真机位置

### 运行项目

运行部署的包, 如已编译仿真则运行以下开启仿真及可视化

```bash
./dexe_robot_server/run_sim.sh
```

如运行w1则需运行 w1_control/ 中的运行脚本


test默认关闭, 如需运行测试则

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test
colcon test-result --verbose
```

---

## 运行参数设置

运行前需在 `/xxx/robot.cfg` 文件中配置参数。 xxx为具体的implement包, 如w1_control, sim_server中等

### 配置参数说明

**robot.cfg** 示例：

```json
{
    "robot_name": "w1_v02_1_with_hand",
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
./style.sh
```