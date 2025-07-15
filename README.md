## 依赖库安装
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc > /dev/null
sudo apt update
sudo apt install nlohmann-json3-dev

### server端需安装
sudo apt install ros-humble-ruckig
sudo apt install ros-humble-pinocchio
sudo apt install ros-humble-control-msgs
注意如果已经通过源码编译安装了则不用再安装ros版本的, 否则会链接冲突

### 源码安装pinocchio
git clone --recursive https://github.com/stack-of-tasks/pinocchio
cd pinocchio
mkdir build
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_PYTHON_INTERFACE=ON \
    -DBUILD_WITH_URDF_SUPPORT=ON \
    -DBUILD_WITH_CASADI_SUPPORT=ON \
    -DBUILD_WITH_AUTODIFF_SUPPORT=ON \
    -DBUILD_UNIT_TESTS=OFF
make -j$(nproc)
sudo make install

源码pinocchio需安装:
sudo apt install -y \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    liburdfdom-tools \
    
### 源码安装hpp-fcl
git clone https://github.com/humanoid-path-planner/hpp-fcl.git
cd hpp-fcl && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install

### 源码安装eigenpy
git clone https://github.com/stack-of-tasks/eigenpy.git
cd eigenpy
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

卸载了ros-humble-pinocchio  ros-humble-hpp-fcl ros-humble-eigenpy
### client端需安装
sudo apt install ros-humble-tf-transformations
pip install meshcat

> **注意：** 以上安装如果出现找不到包的情况检查
sudo vim ～/etc/apt/source.list.d/ros2.list, 源修改为deb [arch=arm64 trusted=yes] http://packages.ros.org/ros2/ubuntu jammy main

### build
colcon build

## 运行参数设置
运行前需在 /robot_models/robot.cfg文件中配置参数

### 配置参数说明

**robot.cfg** 文件示例配置：
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
- `robot_name`: 机器人模型名称，用于在/robot_models中根据urdf建立模型
- `left_arm_name`: (机器人为单臂时可不填)左臂控制器名称
- `right_arm_name`: (机器人为单臂时可不填)右臂控制器名称
- `joints_primary`: (机器人为单臂时可不填)主要关节(左臂)索引数组
- `joints_secondary`: (机器人为单臂时可不填)次要关节索引(右臂)数组

> **注意：** 关节索引从0开始计数，需要根据实际的机器人URDF模型中的关节定义来配置正确的索引值。