# dexe_interfaces

This repository defines the stable ROS 2 Application Programming Interface (API) for DexForce humanoid robots. It contains **only** interface definitions (`.msg`, `.srv`, `.action`) and does not contain any executable code.

## Purpose

This repository serves as the central contract between low-level hardware drivers and high-level application code. It is structured as a meta-package containing a collection of interface packages, which allows for the independent development of different hardware components and their integration into the DexForce ecosystem.

## Repository Structure

The repository is organized into several packages, each containing specific interfaces. The structure is as follows:

```text
dexe_interfaces/
├── chassis_interfaces/             # Interfaces for chassis components.
├── control_gateway_interfaces/     # Main-joint command arbitration interfaces.
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
- `control_gateway_interfaces`: Command envelopes and semantic START/SWITCH/STOP services for main-joint source arbitration.
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
