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
pip install -e .
```

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

#### Run dexsim visualizer
```bash
cd dexe_robot_client
python3 ./examples/run_dexsim_server.py # Modify params according to your config
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