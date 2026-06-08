# Install Dependencies

### DexeRP >= V0.1.0
Downloads from http://192.168.3.120/dexe_release/dexe_robot_planning/ and `pip install`

### OMPL & FCL
```bash
cd dependencies
bash ./install_planner_dependencies.sh
```

### Ros2-humble
导航客户端只需要 ROS2 Python 客户端和 Nav2 接口包，不需要为客户端侧安装完整 `ros-humble-navigation2`：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-nav2-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-srvs \
  ros-humble-rcl-interfaces \
  ros-humble-diagnostic-msgs \
  ros-humble-action-msgs \
  ros-humble-std-msgs
```

避免仅为了运行 `dexe_robot_client` 执行：

```bash
sudo apt install ros-humble-navigation2
```

完整 `ros-humble-navigation2` 会安装 Nav2 server 侧组件，其中 `ros-humble-nav2-smac-planner` 依赖 `ros-humble-ompl`。本仓库的 OMPL 由 `dependencies/install_planner_dependencies.sh` 通过 wheel 安装，混装 apt 版 OMPL 可能带来依赖冲突。只有需要在本机实际启动 Nav2 server 时，才应在对应部署文档中单独安装完整 Nav2 栈。

### [rlia(optional)](http://69.235.177.182:8081/Engine/rlia)
```bash
pip install --no-cache-dir --index-url http://192.168.3.43:8080/simple/ --trusted-host 192.168.3.43 rlia==0.0.0
```

### [dexsim(optional)](http://69.235.177.182:8081/Engine/dexsim)
```bash
pip install --no-cache-dir --index-url http://192.168.3.43:8080/simple/ --trusted-host 192.168.3.43 dexsim==0.0.0
```

### Prebuilt whl package download
http://192.168.3.43:8080/packages/
