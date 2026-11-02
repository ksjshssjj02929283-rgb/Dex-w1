# dexe_interfaces

This repository defines the stable ROS 2 Application Programming Interface (API) for DexForce humanoid robots. It contains **only** interface definitions (`.msg`, `.srv`, `.action`) and does not contain any executable code.

## Purpose

This repository serves as the central contract between low-level hardware drivers and high-level application code. It is structured as a meta-package containing a collection of interface packages, which allows for the independent development of different hardware components and their integration into the DexForce ecosystem.

## Repository Structure

The repository is organized into several packages, each containing specific interfaces. The structure is as follows:

```text
dexe_interfaces/
├── chassis_interfaces/             # Interfaces for chassis components.
├── common_interfaces/              # Interfaces for general robot control like rebooting or diagnostics.
├── gripper_interfaces/             # Interfaces for grippers.
├── hand_interfaces/                # Interfaces for dexterous hands.
├── joint_interfaces/               # All joint-related interfaces.
├── robot_server_interfaces/        # Interfaces for the robot server managing overall state.
├── sensor_interfaces/              # Custom interfaces for non-standard sensors.
├── template_interfaces/            # Template package for creating new interface packages.
├── teleop_interfaces/              # Interfaces for teleoperation configuration read/write.
├── ...
├── .gitignore
└── README.md
```

## Package Structure

Each package in this repository follows a consistent structure:

```text
<package_name>/
├── msg/                             # Contains message definitions (.msg files).
├── srv/                             # Contains service definitions (.srv files).
├── action/                          # Contains action definitions (.action files).
├── package.xml                     # Package metadata.
├── CMakeLists.txt                  # Build configuration for the package.
├── README.md                       # Package documentation.
└── ...
```

## Packages Overview

This repository contains the following interface packages at its root:

- `chassis_interfaces`: Interfaces specific to the chassis, such as wheel encoders and steering.
- `dexe_application_interfaces`: Interfaces for application task execution, status, and diagnostics.
- `common_interfaces`: Interfaces for tasks applicable to multiple hardware components, like rebooting or diagnostics.
- `gripper_interfaces`: Interfaces for grippers, including control and feedback.
- `hand_interfaces`: Interfaces for dexterous hands, including finger control and feedback.
- `joint_interfaces`: All interfaces related to joints, including position, velocity, and torque control.
- `robot_server_interfaces`: Interfaces for the robot server, which manages the overall robot state and provides high-level commands.
- `sensor_interfaces`: Custom interfaces for non-standard sensors. For standard hardware like cameras and Lidars, we use the official `sensor_msgs` package.
- `template_interfaces`: A template package that can be copied to create new interface packages.
- `teleop_interfaces`: Interfaces for teleoperation configuration read/write, exposing `teleop_config.yaml` to X-Wiz via ROS2 services (`TeleopConfigGet` / `TeleopConfigSet`).

## Usage

To use these interfaces, clone this repository into your ROS 2 workspace's `src` folder and build the workspace.

To depend on a package in your own project, add the appropriate dependency to your `package.xml`:

```xml
<depend>template_interfaces</depend>
```

## Creating New Interfaces

If you need to create new interfaces, just copy the `template_interfaces` package and rename it to your desired package name.

```bash
cp -r template_interfaces new_interface_package
```

Make sure to update the `package.xml` and `CMakeLists.txt` files with your new package name and description.

Then, add your `.msg`, `.srv`, or `.action` files in the `msg`, `srv`, or `action` directories respectively. Make sure to follow the naming conventions and structure used in this repository.

## 状态码表校验

运行以下命令递归检查 `supervisor_interfaces/config` 下所有 `.yaml` 和 `.yml` 文件，包含历史码表（也可在命令末尾指定其他文件或目录路径）：

```bash
python3 -m pip install -r utils/requirements-statuscodes.txt
python3 utils/check_statuscodes.py
```

检查要求：

- 根节点只包含非空列表 `modules`；每个模块包含 `module`、`module_name`、`module_id`、`submodules`。
- `submodules` 是非空列表，每项包含 `submodule`、`submodule_name`、`submodule_id`、`status_codes`。
- `status_codes` 是非空列表，每项包含 `status_code`、`description`、`solution`。
- 所有叶子字段必须是非空字符串，禁止 null、空字符串和纯空白；描述文本内部允许空格。
- `module_id` 必须是三位、`submodule_id` 必须是两位、`status_code` 必须是八位 ASCII 数字字符串，数字字段不能含空白，建议全部加引号。
- `module_id` 在整张表中唯一，`submodule_id` 在所属模块内唯一（不同模块可复用），`status_code` 在整张表中唯一；重复时报告当前及首次出现的字段路径。
- `status_code` 前三位必须等于所属模块的 `module_id`，第四至第五位必须等于所属子模块的 `submodule_id`。
- 拒绝重复字段、未知字段、缺失字段和错误层级。只检查，不自动改写编号或补齐内容。

每个文件独立校验，不跨文件判重。某个文件失败后仍继续检查其他文件，最后汇总结果；
任何文件失败或目录内没有 YAML 文件时返回退出码 1，全部通过时返回 0。
GitLab CI 的 `check_statuscodes` 作业会检查上述目录中的所有 YAML 文件。
