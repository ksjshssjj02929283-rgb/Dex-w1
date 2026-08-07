# Robot Server 新功能开发指南

以 TDC（Time Delay Control）力矩控制器的集成为案例，说明在 robot_server 中开发新功能的标准流程。

---

## 总体流程

```
数据结构 -> YAML 配置 -> 配置解析 -> 算法实现 -> 控制器集成 -> 单元测试
```

每一层只依赖下一层。先定义数据结构作为各层之间的契约，再分别展开。

---

## 1. 定义数据结构 (`utils/data_structure.h`)

确定功能需要哪些参数，分为全局和每关节两级。

```cpp
// 每关节配置
struct TimeDelayControlJointConfig {
    double motor_inertia{0.007};
    double m_bar_norm{2.0};
    double stiffness{100.0};
    double damping{20.0};
};

// 全局容器
struct TimeDelayControlConfig {
    std::array<TimeDelayControlJointConfig, ndof_robot> joints{};
};

// 控制器类型枚举
enum class TorqueControllerType { PdController, TdcController };

// 注册到 RobotProperty
struct RobotProperty {
    TimeDelayControlConfig joint_tdc_control;
    TorqueControllerType torque_mode_controller_type{TorqueControllerType::PdController};
};
```

---

## 2. 编写 YAML 配置 (`w1_server/robot_config.yaml`)

YAML 结构与数据结构一一对应。全局参数放版本块顶层，每关节参数放 `joints` 内。

```yaml
w1_v02:
  torque_mode_controller: "pd_controller"   # 全局选择

  joints:
    - name: LEFT_J1
      pd_controller:          # 已有控制器
        stiffness: 900.0
        damping: 70.0
      tdc_controller:         # 新增控制器
        motor_inertia: 0.015
        m_bar_norm: 2.0
        stiffness: 100.0
        damping: 20.0
```

---

## 3. 实现配置解析 (`utils/utils.cpp`)

全局类型用 `magic_enum` 自动映射，每关节参数用带校验的 lambda 读取。

```cpp
// 类型映射: YAML 字符串 -> 枚举
magic_enum::enum_cast<TorqueControllerType>(SnakeCaseToPascalCase(selected));

// 每关节参数解析入口
if (joint_config["tdc_controller"] && joint_config["tdc_controller"].IsMap()) {
    apply_joint_tdc_node(joint_config["tdc_controller"],
                         robot_property.joint_tdc_control.joints[joint_idx]);
}
```

---

## 4. 实现算法类 (`dynamics/joint_time_delay_controller.{h,cpp}`)

放在 `dynamics/` 目录下自动被 CMake 收录。构造接收配置，提供 `UpdateTorqueCommand` 和 `Reset`。

```cpp
class JointTimeDelayController {
public:
    explicit JointTimeDelayController(const TimeDelayControlConfig& config = {});
    bool UpdateTorqueCommand(const VectorDof& cmd_pos, const VectorDof& cmd_vel,
                             const VectorDof& feedback_pos, const VectorDof& feedback_vel,
                             VectorDof& tau_command);
    void Reset();
private:
    VectorDof m_bar_, stiffness_, damping_;
    VectorDof last_feedback_velocity_, last_command_velocity_, output_torque_;
    bool initialized_{false};
    double cycle_time_{torque_ctrl_dt};
};
```

---

## 5. 集成到控制器框架 (`torque_controller/torque_controller_inverse_dynamic.{h,cpp}`)

在 `TorqueControllerInvDynamic` 中注册成员指针，构造时按类型分发，三个状态分支中按类型调用。

```cpp
// 头文件成员声明
TorqueControllerType controller_type_{TorqueControllerType::PdController};
std::shared_ptr<dynamics::JointPdController> joint_pd_controller_{nullptr};
std::shared_ptr<dynamics::JointTimeDelayController> joint_time_delay_controller_{nullptr};
```

---

## 6. 单元测试 (`test/test_joint_time_delay_controller.cpp`)

直接构造 config 结构体，测试首帧、Reset、输出方向等边界条件。

```cpp
TimeDelayControlConfig config;
config.joints[6].motor_inertia = 0.015;
config.joints[6].m_bar_norm = 2.0;
dynamics::JointTimeDelayController controller(config);
```

---

## Checklist

| 步骤 | 目录 | 关键产出 |
|------|------|----------|
| 数据结构 | `utils/data_structure.h` | 配置结构体 + 枚举 + 注册到 `RobotProperty` |
| YAML | `w1_server/robot_config.yaml` | 全局开关 + 每关节参数 |
| 解析 | `utils/utils.cpp` | 类型映射 + 带校验读取 |
| 算法 | `dynamics/` | 控制类声明 + 成员定义 |
| 集成 | `torque_controller/` | 成员指针 + 构造分发 + 三状态调用 |
| 测试 | `test/` | 边界条件 + 物理合理性 |
