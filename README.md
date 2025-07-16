# dexe_robot_server 使用说明

## 依赖库安装

```bash
# ROS2 公钥导入
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc > /dev/null

sudo apt update

# 安装依赖库
sudo apt install nlohmann-json3-dev
sudo apt install ros-humble-pinocchio
sudo apt install clang-format
```

> **注意**：如已通过源码编译安装，无需再安装 ROS 版本，否则会链接冲突。

### ruckig 源码编译安装

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

## 代码风格检查

开发者提交代码前请运行：

```bash
./style.sh
```

---

## 编译项目

```bash
colcon build
```

---

## 运行参数设置

运行前需在 `/robot_models/robot.cfg` 文件中配置参数。

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