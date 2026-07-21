# DexEmbodied Robot Client
## 💻 System
Ubuntu22.04  
Python310

## 📦 Dependencies
Refer to `dependencies/README.md`

## 🚀 Deployment
#### Clone
```bash
git clone http://69.235.177.182:8081/dexembodied/dexe_robot_client.git
cd dexe_robot_client
git submodule update --init --recursive
```

#### Build [dexe_interfaces](http://69.235.177.182:8081/dexembodied/dexe_interfaces.git)
```bash
cd ./dexe_interfaces  
colcon build --packages-select robot_server_interfaces
```
Run beblow before running the main program
```bash
source ./dexe_interfaces/install/setup.bash
```

#### Install dexe_robot_client
```bash
pip install -e . # 安装包
dexe_robot_client-install-scripts # 安装examples和scripts到dexe_robot_client_data中 
```

#### Packaging
```bash
python3 -m build
```
whl包详见`dist`文件夹

#### Uninstall
```bash
pip uninstall dexe_robot_client
```


## 📘 Examples
Deploy `dexe_robot_server(dev)`, refer to [dexe_robot_server](http://69.235.177.182:8081/dexembodied/dexe_robot_server).

Deploy `dexe_devices(main)`, refer to [dexe_devices](http://69.235.177.182:8081/dexembodied/dexe_devices).

#### Run robot server
Automatic startup in PC1.

#### Run sim server
- W1
```bash
cd dexe_robot_server
source ./install/setup.bash
ros2 run sim_server sim_server
```
- EndEffector
```bash
cd dexe_devices
python3 ./brainco_hand/brainco_sim_server.py
# python3 ./pgc_gripper/pgc_sim_server.py
```

#### Run examples
```python
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager

w1_manager = W1Manager(is_sim = True,
                        arm_type=W1ArmType.ANTHROPOMORPHIC, 
                        w1_version=W1Version.V2, 
                        end_effector_type = EndEffectorType.BRAINCO_HAND)

if not w1_manager.connect() and not w1_manager.is_enabled():
    return
    
w1_manager.set_sim_mode(enable_sim = False)
#w1_manager.set_sim_mode(enable_sim = True)
pass
```
More examlpes see `dexe_robot_client/examples`.

## 🧭 Navigation Client

`NavigationClient.set_chassis_control_position(position)` 使用 `control/precise_control` (`nav_interfaces/action/PreciseControl`) action 发送底盘精控目标，不再发布 `/control/ctrl_pose`，也不再调用 `control/get_control_status` 或 `control/cancel_chassis_control` Trigger 服务。

目标仍基于 `base_link`：`position` 为 `[x, 0, yaw]`，`yaw` 单位为度，`x` 和 `yaw` 不能同时非零。控制发送行为与导航一致，不因本地旧 goal handle 或旧状态拦截新目标；`get_control_status()` 像 `get_nav_state()` 一样返回 action feedback/result 维护的本地状态，不消费终态；`cancel_chassis_control()` 通过当前 action goal handle 发送取消请求。

路网导航通过 `/navigate_through_poses` (`nav2_msgs/action/NavigateThroughPoses`) 发送多路点任务，`waypoints` 使用 `[x, y, yaw_rad]`；`get_all_nav_points()` 通过 `/map_asset_manager/get_all_nav_points` (`nav_interfaces/srv/GetAllNavPoints`) 获取点位名称到 `(x, y, theta_rad)` 的映射。


## 🏷️ Others
#### Branches
- Universal functions of the robot are developed on the `main` and `dev` branches
- W1 humanoid is developed on the `main_w1` and `dev_w1` branch and needs to continuously merge the latest changes of `main`
- The feat branch is recommended to be named in the style of `zyn/dev/xxx` and `fsh/w1/xxx`

#### Commit-msg
Refer to [commit message guidelines](https://github.com/angular/angular/blob/main/contributing-docs/commit-message-guidelines.md), including
- feat: A new feature
- fix: A bug fix
- perf: A code change that improves performance
- refactor: A code change that neither fixes a bug nor adds a feature
- docs: Documentation only changes
- test: Adding missing tests or correcting existing tests
- build: Changes that affect the build system or external dependencies (example scopes: gulp, broccoli, npm)
- ci: Changes to our CI configuration files and scripts (examples: CircleCi, SauceLabs)
