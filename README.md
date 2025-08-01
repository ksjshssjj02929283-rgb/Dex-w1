# DexEmbodied Robot Client
# DexEmbodied Robot Client

## 📦 Dependencies
Refer to `dependencies/README.md`

## 🚀 Deployment
#### Clone
```bash
git clone http://69.235.177.182:8081/dexembodied/dexe_robot_client.git
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
Deploy `dexe_robot_server`, refer to [dexe_robot_server](http://69.235.177.182:8081/dexembodied/dexe_robot_server).

#### Run real robot server
todo

#### Run sim robot server with meshcat visualizer
```bash
cd dexe_robot_server
./run_sim.sh
```

#### Run dexsim visualizer
```bash
cd dexe_robot_client
python3 ./examples/run_dexsim_server.py
```

#### Run examples
```bash
cd dexe_robot_client
python3 ./examples/robot_client_example.py
```


## 🏷️ Others
#### Branches
- Universal functions of the robot are developed on the `main` and `dev` branches
- W1 humanoid is developed on the `w1` branch and needs to continuously merge the latest `dev` changes
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

#### format
- python
- cpp

## 📋 TODO
- [ ] Some basic motion API, such as `MoveJi()`, `MoveTX()`, `MoveBY()`, `RotateBZ()`, `RotateTX()`, etc
- [ ] Support `set_user_frame()`
- [ ] Deprecate rlia-related kinematic dependencies
- [ ] Support downloading urdf files from nas
- [ ] Support `end-effector sim_server`
- [ ] Support end-effector in dexsim visualizer
- [ ] SDK packaging