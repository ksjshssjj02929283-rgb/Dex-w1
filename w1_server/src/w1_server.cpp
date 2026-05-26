#include "w1_server.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <set>
#include <yaml-cpp/yaml.h>

#include "robot_server_lib/pinocchio_api.h"

namespace {
constexpr double kDefaultGravity = 9.79;
// 末端加速度/关节加速度低通截止默认值. 与 script/tool_inertia_estimate.py 的 DEFAULT_FILTER_FC_HZ
// 一致 (5 Hz) — 脚本作者已在该工具/传感器组合下离线验证过该频率能跑出物理合法的惯量解.
// 注意: 脚本是双向 filtfilt (零相位 6 阶), 这里是单向二阶临界阻尼 IIR, 衰减更弱、有相位延迟.
// 可以通过 yaml 的 inertia_compensation_filter_hz 覆盖.
constexpr double kDefaultInertiaCompCutoffHz = 5.0;
constexpr double kMinInertiaCompCutoffHz = 0.5;
constexpr double kMaxInertiaCompCutoffHz = 100.0;  // 1 kHz 采样的 10%

bool ReadYamlVector(const YAML::Node& node, int expected_size, Eigen::VectorXd& out) {
    if (!node || !node.IsSequence() || static_cast<int>(node.size()) != expected_size) {
        return false;
    }
    out = Eigen::VectorXd::Zero(expected_size);
    for (int i = 0; i < expected_size; ++i) {
        out[i] = node[i].as<double>();
    }
    return true;
}

// 解析 3x3 惯量矩阵: 期望 YAML 中是 3 行, 每行 3 个 double
bool ReadYamlMatrix3(const YAML::Node& node, Eigen::Matrix3d& out) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        const auto& row = node[i];
        if (!row.IsSequence() || row.size() != 3) {
            return false;
        }
        for (int j = 0; j < 3; ++j) {
            out(i, j) = row[j].as<double>();
        }
    }
    return true;
}
}  // namespace

namespace robot_server_lib {

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim)
    : RobotServerBase(robot_cfg_path, is_sim), line_interpolator_(manager_dt_micro / torque_ctrl_dt_micro) {
    LOG_INFO << "w1_server version: " << W1_SERVER_VERSION;

    // 先初始化ndof_config_, 后续再自动识别模式
    ndof_config_ = ndof_robot;
    sensor_force_.setZero();
    sensor_wrench_.setZero();
    raw_sensor_wrench_.setZero();
    sensor_zero_offset_.setZero();
    LoadToolGravityCompensationConfig(robot_cfg_path);

    // 创建 /feedback/joint 发布器
    feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback/joint", 10);

    // 创建 /imu/chassis 发布器 (替代原有 imu node)
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/chassis", 20);

    // 创建 /feedback/joint_velocity_torque_limit 发布器
    velocity_torque_limit_feedback_publisher_ =
        this->create_publisher<JointVelocityTorqueLimitFeedbackMsg>("/feedback/joint_velocity_torque_limit", 10);

    // 创建 /control/joint 订阅器
    control_subscriber_ = this->create_subscription<JointControlMsg>(
        "/control/joint", 10, std::bind(&RobotServer::ControlCallback, this, std::placeholders::_1));

    // 创建 /control/joint_position 订阅器
    position_control_subscriber_ = this->create_subscription<JointPositionControlMsg>(
        "/control/joint_position", 10, std::bind(&RobotServer::PositionControlCallback, this, std::placeholders::_1));

    // 创建 /control/omni_chassis 订阅器
    omni_chassis_control_subscriber_ = this->create_subscription<ChassisWholeControlMsg>(
        "/control/omni_chassis", 10, std::bind(&RobotServer::OmniChassisControlCallback, this, std::placeholders::_1));

    // 创建 /control/omni_chassis_motors 订阅器
    omni_chassis_motor_control_subscriber_ = this->create_subscription<ChassisMotorControlMsg>(
        "/control/omni_chassis_motors", 10,
        std::bind(&RobotServer::OmniChassisMotorControlCallback, this, std::placeholders::_1));

    // 创建 /control/cmd_vel 订阅器
    diff_chassis_control_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel", 10, std::bind(&RobotServer::DiffChassisControlCallback, this, std::placeholders::_1));
    chassis_shaper_ = std::make_shared<EIInputShaper>(chassis_shaper_type_, DiffChassisNatureFreq, torque_ctrl_dt,
                                                      DiffChassisMaxRampRpm);

    force_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "force_array", rclcpp::QoS(10), std::bind(&RobotServer::ForceArrayCallback, this, std::placeholders::_1));

    // 创建 /control/debug 订阅器
    debug_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/control/debug", 10, std::bind(&RobotServer::DebugCallback, this, std::placeholders::_1));

    // 创建定时器，定时发布feedback 定时器回调函数
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { PublishFeedback(); });

    // 创建定时器，定时发布 IMU 数据 (2ms = 500Hz，与原 imu node 一致)
    imu_timer_ = this->create_wall_timer(std::chrono::milliseconds(2), [this]() { PublishImu(); });

    // 初始化巴特沃斯滤波器和滤波开关
    for (int i = 0; i < ndof_robot; ++i) {
        butterworth_filter_[i] = std::make_shared<ButterworthFilter>(
            this->robot_model_.robot_property_default.butterworth_filter_frequency[i]);
        input_shapers_[i] = std::make_shared<EIInputShaper>(
            input_shaper_type_, this->robot_model_.robot_property_default.input_shaper_frequency[i], torque_ctrl_dt,
            input_shaper_max_ramp_rad_per_sec);
    }

    // 初始化力传感器
    primary_sensor_ = std::make_shared<SriForceSensor>();
    primary_sensor_->SetToolGravityCompensationForZeroCalibration(
        tool_gravity_compensation_enabled_, tool_mass_kg_, tool_com_m_, tool_gravity_magnitude_, tool_model_sign_,
        [this]() -> std::optional<Eigen::Matrix3d> {
            return GetEERotationWorldFromEE(0);
        });
    if (!primary_sensor_->Init(460800)) {
        RCLCPP_WARN(this->get_logger(), "Failed to initialize primary force sensor");
        LOG_WARN << "Failed to initialize primary force sensor";
        primary_sensor_ = nullptr;
    } else {
        Vector6d primary_zero_offset = Vector6d::Zero();
        primary_sensor_->GetZeroOffset(&primary_zero_offset);
        sensor_zero_offset_.head<6>() = primary_zero_offset;
        sensor_zero_offset_valid_[0] = primary_sensor_->HasValidZeroOffset();
        sensor_zero_offset_samples_[0] = primary_sensor_->GetZeroOffsetSampleCount();
        RCLCPP_INFO(this->get_logger(), "Primary force sensor initialized successfully");
        RCLCPP_INFO(this->get_logger(),
                    "Primary force sensor zero offset: valid=%d, samples=%d, [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    sensor_zero_offset_valid_[0], sensor_zero_offset_samples_[0],
                    primary_zero_offset[0], primary_zero_offset[1], primary_zero_offset[2], primary_zero_offset[3],
                    primary_zero_offset[4], primary_zero_offset[5]);
        LOG_INFO << "Primary force sensor initialized successfully";
        LOG_INFO << "Primary force sensor zero offset: valid=" << sensor_zero_offset_valid_[0]
                 << ", samples=" << sensor_zero_offset_samples_[0] << ", offset=" << primary_zero_offset.transpose();
    }
    // secondary_sensor_ = std::make_shared<ForceSensorReader>("/dev/ttyUSB1");  // 暂未使用
    if (!primary_sensor_ && !secondary_sensor_) {
        tool_gravity_compensation_enabled_ = false;
    }
    this->Run();
}

void RobotServer::ForceArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // 示例：若期望 3 维力数据
    if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Force array size %zu < 3", msg->data.size());
        return;
    }
    // 使用数据，例如拷贝前 3 个
    for (size_t i = 0; i < 3; ++i) sensor_force_[i] = msg->data[i];
    RCLCPP_INFO(this->get_logger(), "Received force array: [%f, %f, %f]", sensor_force_[0], sensor_force_[1],
                sensor_force_[2]);
}

void RobotServer::LoadToolGravityCompensationConfig(const std::filesystem::path& robot_cfg_path) {
    tool_gravity_compensation_enabled_ = false;
    tool_mass_kg_ = 0.0;
    tool_gravity_magnitude_ = kDefaultGravity;
    tool_model_sign_ = 1;
    tool_com_m_.setZero();
    tool_inertia_compensation_enabled_ = false;
    tool_inertia_com_.setZero();
    tool_inertia_filter_cutoff_hz_ = kDefaultInertiaCompCutoffHz;
    inertia_primary_joint_id_resolved_ = false;
    for (int i = 0; i < 7; ++i) inertia_primary_joint_id_[i] = -1;
    // 滤波器在解析完截止频率后再构造, 见下方.

    double cutoff_hz = kDefaultInertiaCompCutoffHz;

    try {
        const auto config = YAML::LoadFile(robot_cfg_path.string());
        const auto node = config["tool_gravity_compensation"];
        if (!node) {
            // 无 tool_gravity_compensation 配置时仍要构造默认滤波器 (走 fallthrough)
        } else {
            Eigen::VectorXd com_mm;
            if (!ReadYamlVector(node["com_mm"], 3, com_mm) || !node["mass_kg"]) {
                RCLCPP_WARN(this->get_logger(), "Invalid tool_gravity_compensation config, disabled");
            } else {
                tool_mass_kg_ = node["mass_kg"].as<double>();
                if (node["gravity_magnitude"]) {
                    tool_gravity_magnitude_ = node["gravity_magnitude"].as<double>();
                }
                if (node["model_sign"]) {
                    tool_model_sign_ = node["model_sign"].as<int>();
                }
                tool_com_m_ = com_mm.head<3>() / 1000.0;
                tool_gravity_compensation_enabled_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "Tool gravity compensation enabled, mass_kg=%.4f, gravity=%.4f, model_sign=%d",
                            tool_mass_kg_, tool_gravity_magnitude_, tool_model_sign_);

                // 工具惯量补偿: 必须配置 3x3 inertia_com_kg_m2 才启用; 仅 primary 臂使用.
                // 坐标系语义 (与 script/tool_inertia_estimate.py 输出一致):
                //   - I_C^EE: 关于工具 COM, 在 EE 系基下表达 (kg·m^2)
                //   - com_m_  : 从 EE 原点指向 COM, 在 EE 系下表达 (m)
                // mass≈0 时惯量补偿在物理上无意义 (F=m·a 与 r_C×F 都退化), 同时避免配置错位
                // (mass 错配为 0 但 inertia 非零) 时仍产生 ω×(I·ω) 导致非物理力矩.
                Eigen::Matrix3d inertia_com;
                if (ReadYamlMatrix3(node["inertia_com_kg_m2"], inertia_com)) {
                    if (tool_mass_kg_ < FloatZeroTol) {
                        RCLCPP_WARN(this->get_logger(),
                                    "tool inertia compensation disabled: mass_kg=%.6g is ~0 while "
                                    "inertia_com_kg_m2 is set; fix mass_kg or remove inertia_com_kg_m2",
                                    tool_mass_kg_);
                    } else {
                        tool_inertia_com_ = inertia_com;
                        tool_inertia_compensation_enabled_ = true;
                        RCLCPP_INFO(this->get_logger(),
                                    "Tool inertia compensation enabled (primary). I_C is COM-referenced and "
                                    "expressed in EE frame (kg*m^2); diag=[%.6e, %.6e, %.6e]",
                                    tool_inertia_com_(0, 0), tool_inertia_com_(1, 1), tool_inertia_com_(2, 2));
                    }
                } else if (node["inertia_com_kg_m2"]) {
                    RCLCPP_WARN(this->get_logger(),
                                "Invalid inertia_com_kg_m2 (expect 3x3 list), tool inertia compensation disabled");
                }

                // 滤波器截止频率: 与脚本默认 5 Hz 对齐, 用户可在 yaml 中覆盖.
                // 仅在惯量补偿启用时应用; 校验区间防止误配.
                if (node["inertia_compensation_filter_hz"]) {
                    const double v = node["inertia_compensation_filter_hz"].as<double>();
                    if (!std::isfinite(v) || v < kMinInertiaCompCutoffHz || v > kMaxInertiaCompCutoffHz) {
                        RCLCPP_WARN(this->get_logger(),
                                    "inertia_compensation_filter_hz=%.4f out of range [%.2f, %.2f], using default %.2f Hz",
                                    v, kMinInertiaCompCutoffHz, kMaxInertiaCompCutoffHz, kDefaultInertiaCompCutoffHz);
                    } else {
                        cutoff_hz = v;
                    }
                }
            }
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to load tool gravity compensation config: %s", e.what());
    }

    tool_inertia_filter_cutoff_hz_ = cutoff_hz;
    if (tool_inertia_compensation_enabled_) {
        RCLCPP_INFO(this->get_logger(),
                    "Tool inertia compensation filter cutoff: %.2f Hz (1 kHz sampling, 2nd-order critically damped)",
                    cutoff_hz);
    }
    for (auto& f : ee_accel_filter_) {
        f = std::make_shared<ButterworthFilter>(cutoff_hz);
    }
    for (auto& f : joint_accel_filter_) {
        f = std::make_shared<ButterworthFilter>(cutoff_hz);
    }
    ResetToolInertiaCompensationState();
}


std::optional<Eigen::Matrix3d> RobotServer::GetEERotationWorldFromEE(int arm_index) const {
    // PlannerManager 在 manager_state_ ∈ {Init, Error} 时仍然把 14 维 ee_pose_world_measurement
    // 写入 buffer, 但里面是默认构造的 (0,0,0, 1,0,0,0) —— 看起来 valid 实际是占位 identity.
    // 不加这条门, 力传感器零偏会在 R=I 假设下结算, 残留 ~m·g 的工具重力分量.
    if (this->manager_state_ != Status::Idle && this->manager_state_ != Status::Running) {
        return std::nullopt;
    }
    const auto& ee_pose_world = this->manager_info_.ee_pose_world_measurement;
    const int offset = arm_index * 7;
    if (ee_pose_world.size() < offset + 7) {
        return std::nullopt;
    }

    Eigen::Quaterniond q(ee_pose_world[offset + 3], ee_pose_world[offset + 4], ee_pose_world[offset + 5],
                         ee_pose_world[offset + 6]);
    if (!q.coeffs().allFinite() || q.norm() <= FloatZeroTol) {
        return std::nullopt;
    }
    q.normalize();
    return q.toRotationMatrix();
}

Vector6d RobotServer::ApplyToolGravityCompensation(const Vector6d& wrench_ee, int arm_index, bool* applied) const {
    if (applied) *applied = false;
    if (!tool_gravity_compensation_enabled_) {
        return wrench_ee;
    }

    const auto rotation_world_from_ee_opt = GetEERotationWorldFromEE(arm_index);
    if (!rotation_world_from_ee_opt) {
        return wrench_ee;
    }
    const Eigen::Matrix3d& rotation_world_from_ee = *rotation_world_from_ee_opt;
    const Vector3d gravity_world(0.0, 0.0, -tool_gravity_magnitude_);
    const Vector3d expected_force_ee =
        static_cast<double>(tool_model_sign_) * rotation_world_from_ee.transpose() * (tool_mass_kg_ * gravity_world);

    Vector6d expected_wrench_ee;
    expected_wrench_ee.head<3>() = expected_force_ee;
    expected_wrench_ee.tail<3>() = tool_com_m_.cross(expected_force_ee);

    if (applied) *applied = true;
    return wrench_ee - expected_wrench_ee;
}

Vector6d RobotServer::ApplyToolInertiaCompensationPrimary(const Vector6d& wrench_ee,
                                                          const Eigen::VectorXd& feedback_q_primary,
                                                          const Eigen::VectorXd& feedback_dq_primary,
                                                          bool* applied) {
    if (applied) *applied = false;
    // 防御性早退 (wiring 已做过滤, 这里冗余以便函数本身可独立调用).
    // 早退时写 sentinel 以免 publish 线程读到上一次的有效值.
    auto early_return = [&]() {
        tool_inertia_force_norm_.store(-1.0, std::memory_order_relaxed);
        tool_inertia_torque_norm_.store(-1.0, std::memory_order_relaxed);
    };
    if (!tool_inertia_compensation_enabled_ || !tool_gravity_compensation_enabled_) {
        early_return();
        return wrench_ee;
    }
    if (!robot_model_.primary_arm_model_ptr) {
        early_return();
        return wrench_ee;
    }
    const auto& arm_model = *robot_model_.primary_arm_model_ptr;
    if (arm_model.nv != 7 || feedback_q_primary.size() != 7 || feedback_dq_primary.size() != 7) {
        early_return();
        return wrench_ee;
    }

    constexpr double kDt = torque_ctrl_dt;  // 1 ms

    // 注意: pinocchio::GetJacobiMatrix / GetFK 内部仍会构造 Data 与动态矩阵.
    // 这是 robot_server_lib 当前 API 的限制 (cartesian_impedance_controller 等
    // 1 kHz 控制器也是这样调用), 留作 lib 层后续优化项. 这里至少避免在表达式
    // 路径上再次产生动态中间结果.
    const Eigen::MatrixXd J_armbase_dyn =
        pinocchio::GetJacobiMatrix(arm_model, feedback_q_primary);
    if (J_armbase_dyn.rows() != 6 || J_armbase_dyn.cols() != 7) {
        early_return();
        return wrench_ee;
    }
    const Eigen::Matrix<double, 6, 7> J_armbase = J_armbase_dyn;
    const Eigen::Matrix<double, 7, 1> dq7 = feedback_dq_primary.head<7>();
    const Eigen::Matrix<double, 6, 1> twist_armbase = J_armbase * dq7;
    const Vector3d v_ee_armbase = twist_armbase.head<3>();
    const Vector3d w_ee_armbase = twist_armbase.tail<3>();

    // 首拍: 仅记录历史并暖滤波器 (Apply(0) 让滤波器以 0 作为初值开始), 直接跳过本周期补偿.
    if (tool_inertia_first_sample_) {
        for (int i = 0; i < 3; ++i) ee_accel_filter_[i]->Apply(0.0);
        for (int i = 0; i < 7; ++i) joint_accel_filter_[i]->Apply(0.0);
        prev_ee_linvel_armbase_ = v_ee_armbase;
        prev_dq_primary_ = dq7;
        tool_inertia_first_sample_ = false;
        tool_inertia_force_norm_.store(-1.0, std::memory_order_relaxed);
        tool_inertia_torque_norm_.store(-1.0, std::memory_order_relaxed);
        return wrench_ee;
    }

    // 末端线加速度: 对 v_ee 做后向差分, 再 Butterworth 低通滤波
    Vector3d a_ee_armbase;
    {
        const Vector3d a_raw = (v_ee_armbase - prev_ee_linvel_armbase_) / kDt;
        for (int i = 0; i < 3; ++i) {
            a_ee_armbase[i] = ee_accel_filter_[i]->Apply(a_raw[i]);
        }
    }
    // 关节加速度: 对 dq 做后向差分, 再 Butterworth 低通滤波
    // (按用户要求, α 由 feedback dq 微分再过滤得到)
    Eigen::Matrix<double, 7, 1> ddq_filtered;
    {
        const Eigen::Matrix<double, 7, 1> ddq_raw = (dq7 - prev_dq_primary_) / kDt;
        for (int i = 0; i < 7; ++i) {
            ddq_filtered[i] = joint_accel_filter_[i]->Apply(ddq_raw[i]);
        }
    }
    prev_ee_linvel_armbase_ = v_ee_armbase;
    prev_dq_primary_ = dq7;

    // α (末端角加速度, arm-base 对齐) = J_omega * ddq_filtered
    const Vector3d alpha_ee_armbase = J_armbase.bottomRows<3>() * ddq_filtered;

    // EE 旋转 (从 EE 到 arm-base): 直接由 arm-only 模型的 FK 取得.
    // 注意 GetEERotationWorldFromEE 返回的是 global<-ee, 对于 W1 这与 armbase<-ee 一般不同
    // (左右臂 base 相对于躯干有固定旋转), 因此不能复用.
    const Eigen::Matrix4d T_armbase_ee = pinocchio::GetFK(arm_model, feedback_q_primary);
    const Eigen::Matrix3d R_a_e = T_armbase_ee.topLeftCorner<3, 3>();

    // 工具 COM 在 arm-base 坐标的偏移 (从 EE 原点到 COM)
    const Vector3d r_C_armbase = R_a_e * tool_com_m_;

    // COM 加速度 (arm-base): a_C = a_ee + α × r_C + ω × (ω × r_C)
    const Vector3d a_C_armbase =
        a_ee_armbase + alpha_ee_armbase.cross(r_C_armbase) + w_ee_armbase.cross(w_ee_armbase.cross(r_C_armbase));

    // COM 处的惯量矩阵在 arm-base: I_C_a = R * I_C * R^T
    const Eigen::Matrix3d I_C_armbase = R_a_e * tool_inertia_com_ * R_a_e.transpose();

    // 工具产生的反作用力旋量 (arm-base 坐标, 等效在 EE 原点处):
    //   F_a = m * a_C
    //   τ_a (about EE origin) = I_C_a * α + ω × (I_C_a * ω) + r_C × (m * a_C)
    const Vector3d F_armbase = tool_mass_kg_ * a_C_armbase;
    const Vector3d tau_about_ee_armbase =
        I_C_armbase * alpha_ee_armbase + w_ee_armbase.cross(I_C_armbase * w_ee_armbase) + r_C_armbase.cross(F_armbase);

    // 旋到 EE 坐标, 应用 model_sign
    Vector6d expected_inertia_wrench_ee;
    expected_inertia_wrench_ee.head<3>() = static_cast<double>(tool_model_sign_) * (R_a_e.transpose() * F_armbase);
    expected_inertia_wrench_ee.tail<3>() =
        static_cast<double>(tool_model_sign_) * (R_a_e.transpose() * tau_about_ee_armbase);

    // 诊断: 暴露本周期实际扣除的惯量 wrench 范数 (EE 系) 给 publish 线程.
    tool_inertia_force_norm_.store(expected_inertia_wrench_ee.head<3>().norm(), std::memory_order_relaxed);
    tool_inertia_torque_norm_.store(expected_inertia_wrench_ee.tail<3>().norm(), std::memory_order_relaxed);

    if (applied) *applied = true;
    return wrench_ee - expected_inertia_wrench_ee;
}

void RobotServer::ResetToolInertiaCompensationState() {
    tool_inertia_first_sample_ = true;
    prev_ee_linvel_armbase_.setZero();
    prev_dq_primary_.setZero();
    for (auto& f : ee_accel_filter_) {
        if (f) f->Reset();
    }
    for (auto& f : joint_accel_filter_) {
        if (f) f->Reset();
    }
    // 诊断 sentinel: -1 表示本周期未运行 (publish 线程据此区分 "0 N" 与 "未跑")
    tool_inertia_force_norm_.store(-1.0, std::memory_order_relaxed);
    tool_inertia_torque_norm_.store(-1.0, std::memory_order_relaxed);
}

void RobotServer::TorqueControlThreadFunc() {
    if (!is_sim_) {
        Setup_Realtime();
    }
    TorqueControllerManager torque_controller_manager(this->robot_model_, this->manager_buffers_, this->robot_buffers_);
    // 初始化共享内存
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto& control_shm_instance = ControlSharedMemory::GetInstance();

    Int64 loop_counter = 0;
    this->is_robot_ready_ = false;
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    double usage_sum = 0.0;
    double peak_usage = 0.0;
    int usage_count = 0;

    while (rclcpp::ok()) {
        loop_counter++;
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);
        this->manager_buffers_.planner_manager_info->PeekLatest(&this->manager_info_);

        auto t_start = std::chrono::steady_clock::now();
        // 力传感器零偏延迟结算: 等到 EE 位姿可用后才完成校准并去除工具重力, 然后镜像 ready 标志.
        if (primary_sensor_) {
            if (primary_sensor_->TryFinalizeBias()) {
                RCLCPP_INFO(this->get_logger(),
                            "Primary force sensor zero offset finalized; samples=%d",
                            primary_sensor_->GetZeroOffsetSampleCount());
                LOG_INFO << "Primary force sensor zero offset finalized; samples="
                         << primary_sensor_->GetZeroOffsetSampleCount();
            }
            primary_sensor_ready_ = primary_sensor_->HasValidZeroOffset();
        } else {
            primary_sensor_ready_ = false;
        }
        if (secondary_sensor_) {
            if (secondary_sensor_->TryFinalizeBias()) {
                RCLCPP_INFO(this->get_logger(),
                            "Secondary force sensor zero offset finalized; samples=%d",
                            secondary_sensor_->GetZeroOffsetSampleCount());
                LOG_INFO << "Secondary force sensor zero offset finalized; samples="
                         << secondary_sensor_->GetZeroOffsetSampleCount();
            }
            secondary_sensor_ready_ = secondary_sensor_->HasValidZeroOffset();
        } else {
            secondary_sensor_ready_ = false;
        }

        // 读取力传感器数据
        Vector6d sensor_data_primary = Vector6d::Zero();
        Vector6d sensor_data_secondary = Vector6d::Zero();
        Vector6d zeroed_sensor_data_primary = Vector6d::Zero();
        Vector6d zeroed_sensor_data_secondary = Vector6d::Zero();
        Vector6d raw_sensor_data_primary = Vector6d::Zero();
        Vector6d raw_sensor_data_secondary = Vector6d::Zero();
        const bool primary_wrench_valid =
            primary_sensor_ && primary_sensor_ready_ && primary_sensor_->PeekLatest(&zeroed_sensor_data_primary);
        const bool secondary_wrench_valid =
            secondary_sensor_ && secondary_sensor_ready_ && secondary_sensor_->PeekLatest(&zeroed_sensor_data_secondary);
        if (primary_sensor_ && primary_sensor_ready_) {
            primary_sensor_->PeekLatestRaw(&raw_sensor_data_primary);
            Vector6d primary_zero_offset = Vector6d::Zero();
            primary_sensor_->GetZeroOffset(&primary_zero_offset);
            sensor_zero_offset_.head<6>() = primary_zero_offset;
            sensor_zero_offset_valid_[0] = primary_sensor_->HasValidZeroOffset();
            sensor_zero_offset_samples_[0] = primary_sensor_->GetZeroOffsetSampleCount();
        }
        if (secondary_sensor_ && secondary_sensor_ready_) {
            secondary_sensor_->PeekLatestRaw(&raw_sensor_data_secondary);
            Vector6d secondary_zero_offset = Vector6d::Zero();
            secondary_sensor_->GetZeroOffset(&secondary_zero_offset);
            sensor_zero_offset_.tail<6>() = secondary_zero_offset;
            sensor_zero_offset_valid_[1] = secondary_sensor_->HasValidZeroOffset();
            sensor_zero_offset_samples_[1] = secondary_sensor_->GetZeroOffsetSampleCount();
        }
        // 工具补偿状态: 用于"启用了但本周期未扣到"的 force-control 安全策略 (见下方清零逻辑).
        bool gravity_applied_primary = false;
        bool gravity_applied_secondary = false;
        bool inertia_applied_primary = false;

        if (primary_wrench_valid) {
            sensor_data_primary =
                ApplyToolGravityCompensation(zeroed_sensor_data_primary, 0, &gravity_applied_primary);
        }
        if (secondary_wrench_valid) {
            sensor_data_secondary =
                ApplyToolGravityCompensation(zeroed_sensor_data_secondary, 1, &gravity_applied_secondary);
        }

        // 叠加手动输入的力数据；没有六维力传感器时保持控制律输入 wrench 为零。
        if (primary_wrench_valid) {
            sensor_data_primary.head<3>() += sensor_force_;
        }

        // 使用临时变量接收共享内存数据
        RobotFeedback feedback_temp{};
        bool ret = feedback_shm_instance.PeekLatest(&feedback_temp) && feedback_temp.cnt > 0;

        // 根据模式排序反馈数据到ndof_robot个关节标准顺序
        if (ret) {
            if (!robot_mode_auto_detected_ && RobotMode::Unknown != DetectRobotModeFromFeedback(feedback_temp)) {
                robot_mode_auto_detected_ = true;
            }

            if (SortFeedbackByJointName(feedback_temp)) {
                robot_feedback_ = feedback_temp;
            } else {
                ret = false;
            }
        } else {
            robot_feedback_.robot_status_word = 0;
            for (int i = 0; i < MAX_JOINT_NUM; ++i) {
                robot_feedback_.al_status[i] = 0;
                robot_feedback_.joint_feedback[i].joint_error_code = 0;
                robot_feedback_.joint_feedback[i].joint_temperature = 0;
            }
        }

        // 工具惯量补偿 (仅 primary): 在重力补偿之后, 在 sensor_data_primary 写入测量之前.
        // 用单一布尔门控所有前置条件, 防止后续修改任一条件时漏掉 Reset 路径.
        if (tool_inertia_compensation_enabled_) {
            // 首次解析 primary 7 个关节在 whole-body 模型下的 joint id, 之后从缓存读.
            // 失败则永久关闭惯量补偿 (URDF/yaml 不匹配是配置错, 不是运行时偶发).
            if (!inertia_primary_joint_id_resolved_) {
                const auto& wb_model = *robot_model_.model_ptr;
                const auto& primary_names = robot_model_.params_ptr->joints_primary_names;
                bool ok = (primary_names.size() == 7);
                for (int i = 0; ok && i < 7; ++i) {
                    const int joint_id = wb_model.getJointId(primary_names[i]) - 1;
                    if (joint_id < 0 || joint_id >= ndof_robot) {
                        ok = false;
                        break;
                    }
                    inertia_primary_joint_id_[i] = joint_id;
                }
                inertia_primary_joint_id_resolved_ = ok;
                if (!ok) {
                    RCLCPP_WARN(this->get_logger(),
                                "tool inertia compensation: failed to resolve primary joint ids, disabling");
                    tool_inertia_compensation_enabled_ = false;
                }
            }

            const bool should_run_inertia =
                ret && primary_wrench_valid && tool_inertia_compensation_enabled_ &&
                inertia_primary_joint_id_resolved_;

            if (should_run_inertia) {
                for (int i = 0; i < 7; ++i) {
                    const int jid = inertia_primary_joint_id_[i];
                    inertia_q_primary_buf_[i] = robot_feedback_.joint_feedback[jid].joint_position;
                    inertia_dq_primary_buf_[i] = robot_feedback_.joint_feedback[jid].joint_velocity;
                }
                sensor_data_primary = ApplyToolInertiaCompensationPrimary(
                    sensor_data_primary, inertia_q_primary_buf_, inertia_dq_primary_buf_,
                    &inertia_applied_primary);
            } else {
                // 任一前置条件不满足都重置滤波器与历史, 避免下次有效样本带入旧状态
                ResetToolInertiaCompensationState();
            }
        }

        // 力控安全策略: 已启用但本周期未成功扣减的补偿视为"失效", 把对应臂的 sensor_data 清零,
        // 让下游 JointImpedanceController::ComputeExternalTorque 的 isZero 短路使 torque_ext=0,
        // 避免把"工具自重 / 惯性反作用"误当外力反馈进控制律.
        // 注 1: "未启用" (用户没配 inertia_com_kg_m2 / 没装 secondary 传感器等) 不视为失效.
        // 注 2: 清零会同时清掉 force_array 话题手动注入的 sensor_force_, 这是有意的——
        //       补偿失效时 sensor_data_primary 已被工具自重污染, 在它上面叠加注入力没有意义.
        const bool primary_compensation_failed =
            primary_wrench_valid &&
            ((tool_gravity_compensation_enabled_ && !gravity_applied_primary) ||
             (tool_inertia_compensation_enabled_ && !inertia_applied_primary));
        const bool secondary_compensation_failed =
            secondary_wrench_valid && tool_gravity_compensation_enabled_ && !gravity_applied_secondary;
        if (primary_compensation_failed) {
            sensor_data_primary.setZero();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "primary tool compensation lapsed (gravity_applied=%d, inertia_applied=%d); "
                                 "zeroing sensor_data_primary so torque_ext=0 in joint impedance",
                                 gravity_applied_primary, inertia_applied_primary);
        }
        if (secondary_compensation_failed) {
            sensor_data_secondary.setZero();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "secondary tool gravity compensation lapsed; zeroing sensor_data_secondary");
        }

        // 根据状态写入机器人控制字
        this->is_robot_ready_ = false;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON;
        if (ret) {
            this->SwitchControlWord();
        }

        auto t_feedback = std::chrono::steady_clock::now();

        // 如果开启了记录, 则在独立的记录日志中记录数据
        if (this->manager_info_.robot_property.enable_record && manager_state_ == Status::Running &&
            loop_counter % 1 == 0) {
            int index_offset = GetFirstJointIndex();
            double timestamp_s = this->now().nanoseconds() / 1e9;

            // 从 JointFeedback 中提取字段用于日志记录
            std::array<double, MAX_JOINT_NUM> position_log_arr_{};
            std::array<double, MAX_JOINT_NUM> velocity_log_arr_{};
            std::array<double, MAX_JOINT_NUM> torque_log_arr_{};
            std::array<double, MAX_JOINT_NUM> position_cmd_log_arr_{};
            std::array<double, MAX_JOINT_NUM> torque_ff_log_arr_{};
            for (int i = 0; i < ndof_robot; ++i) {
                position_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_position;
                velocity_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_velocity;
                torque_log_arr_[i] = robot_feedback_.joint_feedback[i].joint_torque;
                position_cmd_log_arr_[i] = robot_control_.joint_control[i].joint_position;
                torque_ff_log_arr_[i] = robot_control_.joint_control[i].joint_torque_feedforward;
            }

            nanolog::NanoLogLine logline(nanolog::LogLevel::CRIT, __FILE__, __func__, __LINE__, false);
            logline << "{\"timestamp\": \"" << std::to_string(timestamp_s) << "\", "
                    << "\"joint_position\": "
                    << utils::VectorToJsonArrayString(position_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_velocity\": "
                    << utils::VectorToJsonArrayString(velocity_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_torque\":"
                    << utils::VectorToJsonArrayString(torque_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_position_input\": "
                    << utils::VectorToJsonArrayString(joint_position_input_, index_offset, ndof_config_).str()
                    << ", \"joint_position_after_ruckig\": "
                    << utils::VectorToJsonArrayString(joint_position_before_filter_, index_offset, ndof_config_).str()
                    << ", \"joint_position_after_filter\": "
                    << utils::VectorToJsonArrayString(joint_position_after_filter_, index_offset, ndof_config_).str()
                    << ", \"joint_position_cmd\": "
                    << utils::VectorToJsonArrayString(position_cmd_log_arr_, index_offset, ndof_config_).str()
                    << ", \"joint_torque_feedforward\": "
                    << utils::VectorToJsonArrayString(torque_ff_log_arr_, index_offset, ndof_config_).str() << "}";
            nanolog::LogManager::instance().write_to_logger("record", std::move(logline));
        }

        // 如果机器人就绪，则写入测量数据
        if (this->is_robot_ready_) {
            // 关节反馈数据写入测量数据
            JointRawMeasurement raw_measurement{};
            raw_measurement.sensor_wrench = sensor_wrench_ =
                utils::GetSensorWrench(sensor_data_primary, sensor_data_secondary);
            raw_sensor_wrench_ = utils::GetSensorWrench(raw_sensor_data_primary, raw_sensor_data_secondary);

            for (int i = 0; i < ndof_robot; ++i) {
                raw_measurement.position[i] = robot_feedback_.joint_feedback[i].joint_position;
                raw_measurement.velocity[i] = robot_feedback_.joint_feedback[i].joint_velocity;
                raw_measurement.effort[i] = robot_feedback_.joint_feedback[i].joint_torque;
                // 检测力矩限制是否达到阈值
                if (robot_feedback_.joint_feedback[i].joint_torque >=
                        robot_control_.joint_control[i].joint_positive_torque_limit * 0.9 ||
                    robot_feedback_.joint_feedback[i].joint_torque <=
                        -robot_control_.joint_control[i].joint_negative_torque_limit * 0.9) {
                    is_torque_limit_reached_[i] = true;
                } else {
                    is_torque_limit_reached_[i] = false;
                }
                raw_measurement.time_point = TimePointSystem::Now();
            }
            this->robot_buffers_.joint_raw_measurement->AppendNewElement(raw_measurement);
        } else {
            this->robot_buffers_.joint_raw_measurement->Reset();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Robot is not ready, shm status: %d, statusword: %s", ret,
                                 utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str());
            LOG_CRIT_THROTTLE(5000) << "Robot is not ready, shm status: " << ret
                                    << ", statusword: " << robot_feedback_.robot_status_word;
        }
        auto t_log = std::chrono::steady_clock::now();

        // 力控计算处理
        torque_controller_manager.LoopOnce();

        auto t_torque = std::chrono::steady_clock::now();

        // 获取并写入控制数据: 关节位置, 速度, 关节力矩限制, 外部力矩
        TorqueCtrlCommand torque_cmd{};
        this->planner_cmd_ = PlannerManagerCommand{};  // 默认值，避免未写入时使用了旧数据
        this->is_cmd_valid_ = false;
        this->manager_buffers_.planner_manager_info->PeekLatest(&this->manager_info_);
        if (this->manager_buffers_.planner_manager_command->PeekLatest(&this->planner_cmd_) &&
            this->robot_buffers_.torque_controller_command->PeekLatest(&torque_cmd)) {
            this->is_cmd_valid_ = true;
            // 位置指令线性插值
            VectorDof interpolated_position = line_interpolator_.Interpolate(this->planner_cmd_.cmd.position);
            JointMeasurement manager_measurement{};
            for (int i = 0; i < ndof_robot; ++i) {
                this->robot_control_.joint_control[i].joint_position = interpolated_position[i];
                this->robot_control_.joint_control[i].joint_velocity = this->planner_cmd_.cmd.velocity[i];
                this->robot_control_.joint_control[i].joint_positive_torque_limit =
                    robot_model_.params_ptr->joint_torque_limit[i];
                this->robot_control_.joint_control[i].joint_negative_torque_limit =
                    robot_model_.params_ptr->joint_torque_limit[i];
                switch (this->manager_info_.robot_property.run_mode) {
                    case RunMode::Position:
                        if ((previous_mode_ != this->manager_info_.robot_property.run_mode) &&
                            (switch_counter_ < max_switch_count_) && this->robot_model_.IsDynamicParamsAvailable()) {
                            // 切换之前执行位置保持
                            if (i == 0) {
                                switch_counter_++;
                            }
                            // 位置等于反馈位置
                            if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                                if (IsArmJoint(i)) {
                                    manager_measurement.q_desired_pre[i] =
                                        robot_feedback_.joint_feedback[i].joint_position;
                                    this->robot_control_.joint_control[i].joint_position =
                                        robot_feedback_.joint_feedback[i].joint_position;
                                    line_interpolator_.Reset();
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            this->robot_control_.joint_control[i].joint_torque_feedforward =
                                torque_cmd.torque_feedforward[i];
                            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        } else {
                            switch_counter_ = 0;
                            previous_mode_ = this->manager_info_.robot_property.run_mode;
                            // 更新关节力矩限制
                            this->robot_control_.joint_control[i].joint_positive_torque_limit = std::clamp(
                                torque_cmd.torque_feedforward[i] + robot_model_.params_ptr->joint_torque_limit[i] *
                                                                       this->manager_info_.robot_property.safe_level,
                                0.1 * robot_model_.params_ptr->joint_torque_limit[i],
                                robot_model_.params_ptr->joint_torque_limit[i]);
                            this->robot_control_.joint_control[i].joint_negative_torque_limit = std::clamp(
                                -torque_cmd.torque_feedforward[i] + robot_model_.params_ptr->joint_torque_limit[i] *
                                                                        this->manager_info_.robot_property.safe_level,
                                0.1 * robot_model_.params_ptr->joint_torque_limit[i],
                                robot_model_.params_ptr->joint_torque_limit[i]);
                            this->robot_control_.joint_control[i].joint_torque_feedforward =
                                torque_cmd.torque_feedforward[i];
                        }
                        break;
                    case RunMode::Torque:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.joint_control[i].joint_torque_feedforward = torque_cmd.torque[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        break;
                    case RunMode::Drag:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.joint_control[i].joint_torque_feedforward =
                            torque_cmd.torque_feedforward[i];
                        if (this->robot_model_.IsDynamicParamsAvailable()) {
                            // 位置等于反馈位置
                            if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                                if (IsArmJoint(i)) {
                                    manager_measurement.q_desired_pre[i] =
                                        robot_feedback_.joint_feedback[i].joint_position;
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        } else {
                            // 位置等于反馈位置
                            if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                                if (IsArmJoint(i) && is_torque_limit_reached_[i]) {
                                    manager_measurement.q_desired_pre[i] =
                                        robot_feedback_.joint_feedback[i].joint_position;
                                }
                                manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                            }
                            // 更新关节力矩限制
                            if (torque_cmd.torque_feedforward[i] >= 0) {
                                robot_control_.joint_control[i].joint_positive_torque_limit =
                                    std::clamp(torque_cmd.torque_feedforward[i], 0.0,
                                               robot_model_.params_ptr->joint_torque_limit[i]);
                                robot_control_.joint_control[i].joint_negative_torque_limit = 0.0;
                            } else {
                                robot_control_.joint_control[i].joint_positive_torque_limit = 0.0;
                                robot_control_.joint_control[i].joint_negative_torque_limit =
                                    std::clamp(-torque_cmd.torque_feedforward[i], 0.0,
                                               robot_model_.params_ptr->joint_torque_limit[i]);
                            }
                        }
                        break;
                    case RunMode::CartesianImpedance:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.joint_control[i].joint_torque_feedforward = torque_cmd.torque_impedance[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        break;
                    case RunMode::DragImpedance:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        this->robot_control_.joint_control[i].joint_torque_feedforward = torque_cmd.torque[i];
                        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                        // 位置等于反馈位置
                        if (manager_buffers_.joint_measurement->PeekLatest(&manager_measurement)) {
                            if (IsArmJoint(i)) {
                                manager_measurement.q_desired_pre[i] = robot_feedback_.joint_feedback[i].joint_position;
                            }
                            manager_buffers_.joint_measurement->AppendNewElement(manager_measurement);
                        }
                        break;
                    case RunMode::Sleep:
                    case RunMode::QuickStop:
                        previous_mode_ = this->manager_info_.robot_property.run_mode;
                        break;
                    default:
                        assert(false && "Unknown RunMode");
                        break;
                }
                joint_position_before_filter_[i] = this->robot_control_.joint_control[i].joint_position;
                // 对所有关节应用巴特沃斯滤波 (在 EI 输入整形之前)
                if (this->manager_info_.robot_property.enable_butterworth_filter[i]) {
                    robot_control_.joint_control[i].joint_position =
                        butterworth_filter_[i]->Apply(robot_control_.joint_control[i].joint_position);
                } else {
                    butterworth_filter_[i]
                        ->Reset();  // 如果当前关节不启用滤波器，则重置滤波器状态，避免下次启用时的突变
                }
                joint_position_after_filter_[i] = this->robot_control_.joint_control[i].joint_position;

                // 对所有关节应用 EI 输入整形
                if (this->manager_info_.robot_property.enable_input_shaper[i]) {
                    robot_control_.joint_control[i].joint_position =
                        input_shapers_[i]->Apply(robot_control_.joint_control[i].joint_position);
                } else {
                    input_shapers_[i]->Reset();  // 如果当前关节不启用输入整形，则重置状态
                }
            }
        } else {
            line_interpolator_.Reset();
            for (int i = 0; i < ndof_robot; ++i) {
                butterworth_filter_[i]->Reset();  // 重置滤波器状态
                input_shapers_[i]->Reset();       // 重置输入整形器状态
            }
        }
        // 如果状态字发生变化了则打印
        static int16_t last_status_word_ = -1;
        static int16_t last_control_word_ = -1;
        if (robot_feedback_.robot_status_word != last_status_word_ ||
            robot_control_.robot_control_word != last_control_word_) {
            RCLCPP_INFO(this->get_logger(), "Status word changed to: %s, control word changed to %s",
                        utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str(),
                        utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str().c_str());
            LOG_INFO << "";
            LOG_INFO << "Status word changed to "
                     << utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str()
                     << ", control word changed to "
                     << utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str()
                     << ", robot_server_state:" << GetRobotServerStateMsg().data.c_str();
            last_status_word_ = robot_feedback_.robot_status_word;
            last_control_word_ = robot_control_.robot_control_word;
        }

        // 底盘控制：应用输入整形并写入共享内存
        if (robot_mode_ == RobotMode::W1) {
            robot_control_.chassis_control.chassis_mode = chassis_control_target_.chassis_mode;
            robot_control_.chassis_control.cnt++;

            if (chassis_control_target_.chassis_mode == 1) {
                // 差速模式：对左右轮速度应用输入整形
                if (this->manager_info_.robot_property.enable_chassis_shaper) {
                    auto [shaper_left, shaper_right] = chassis_shaper_->Apply(
                        chassis_control_target_.chassis_left_velocity, chassis_control_target_.chassis_right_velocity);
                    robot_control_.chassis_control.chassis_left_velocity = shaper_left;
                    robot_control_.chassis_control.chassis_right_velocity = shaper_right;
                } else {
                    chassis_shaper_->Reset();
                    robot_control_.chassis_control.chassis_left_velocity =
                        chassis_control_target_.chassis_left_velocity;
                    robot_control_.chassis_control.chassis_right_velocity =
                        chassis_control_target_.chassis_right_velocity;
                }
            } else if (chassis_control_target_.chassis_mode == 2) {
                // 全向电机控制模式：直接复制
                for (size_t i = 0; i < OMNI_MOTOR_NUM; i++) {
                    robot_control_.chassis_control.chassis_motor_velocity[i] =
                        chassis_control_target_.chassis_motor_velocity[i];
                    robot_control_.chassis_control.chassis_motor_direction[i] =
                        chassis_control_target_.chassis_motor_direction[i];
                }
                robot_control_.chassis_control.chassis_estop = chassis_control_target_.chassis_estop;
                robot_control_.chassis_control.chassis_neutral = chassis_control_target_.chassis_neutral;
            } else if (chassis_control_target_.chassis_mode == 0) {
                RCLCPP_WARN_ONCE(this->get_logger(), "Whole chassis mode is not supported");
            }
        }
        RobotControl robot_control_temp = robot_control_;
        SortControlByJointName(robot_control_temp);
        robot_control_temp.timestamp = this->now().nanoseconds();
        control_shm_instance.AppendNewElement(robot_control_temp);

        auto now_time = std::chrono::steady_clock::now();
        double current_usage =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now_time - t_start).count()) /
            torque_ctrl_dt_micro * 100.0;
        usage_sum += current_usage;
        usage_count++;
        if (current_usage > peak_usage) {
            peak_usage = current_usage;
        }

        static nanolog::ThrottleState stats_throttle;
        if (stats_throttle.should_log(5000)) {
            RCLCPP_INFO(this->get_logger(), "Avg Usage(last 5s): %.2f%%, Peak Usage: %.2f%%",
                        (usage_sum / std::max(1, usage_count)), peak_usage);
            LOG_INFO << "Avg Usage(last 5s): " << (usage_sum / std::max(1, usage_count)) << "%, Peak: " << peak_usage
                     << "%";
            usage_sum = 0.0;
            usage_count = 0;
            peak_usage = 0.0;
        }

        if (now_time > next_cycle_time_point) {
            auto expected_start = next_cycle_time_point - std::chrono::microseconds(torque_ctrl_dt_micro);
            auto elapsed_micro =
                std::chrono::duration_cast<std::chrono::microseconds>(now_time - expected_start).count();
            double usage = static_cast<double>(elapsed_micro) / torque_ctrl_dt_micro * 100.0;
            auto dt_feedback = std::chrono::duration_cast<std::chrono::microseconds>(t_feedback - t_start).count();
            auto dt_log = std::chrono::duration_cast<std::chrono::microseconds>(t_log - t_feedback).count();
            auto dt_torque = std::chrono::duration_cast<std::chrono::microseconds>(t_torque - t_log).count();
            auto dt_cmd = std::chrono::duration_cast<std::chrono::microseconds>(now_time - t_torque).count();
            LOG_WARN_THROTTLE(1000) << "Torque control thread cycle time exceeded 100%! Usage: " << usage
                                    << "%. Breakdown(us): Feedback=" << dt_feedback << ", Log=" << dt_log
                                    << ", Torque=" << dt_torque << ", CmdAndWrite=" << dt_cmd;
            next_cycle_time_point = now_time;
        }

        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServer::SwitchControlWord() {
    static bool inited = false;
    // 0. Sleep/QuickStop 模式下进入QUICK_STOP，阻止写入measurement
    if (this->manager_info_.robot_property.run_mode == RunMode::Sleep ||
        this->manager_info_.robot_property.run_mode == RunMode::QuickStop) {
        this->is_robot_ready_ = false;
        line_interpolator_.Reset();
        robot_control_.robot_control_word = CTRLWORD_MOTOR_QUICK_STOP;
        for (int i = 0; i < ndof_robot; i++) {
            this->robot_control_.joint_control[i].joint_position =
                this->robot_feedback_.joint_feedback[i].joint_position;
            this->robot_control_.joint_control[i].joint_positive_torque_limit =
                robot_model_.params_ptr->joint_torque_limit[i];
            this->robot_control_.joint_control[i].joint_negative_torque_limit =
                robot_model_.params_ptr->joint_torque_limit[i];
        }
        return;
    }
    // 1. 根据状态字切换控制字
    switch (robot_feedback_.robot_status_word) {
        case STATUSWORD_MOTOR_READY:
            // ready状态is_robot_ready_置为true, 可以写入measurement, 生成cmd后再使能电机
            this->is_robot_ready_ = true;
            inited = true;
            if (this->is_cmd_valid_) {
                this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
            } else {
                line_interpolator_.Reset();
                for (int i = 0; i < ndof_robot; i++) {
                    this->robot_control_.joint_control[i].joint_position =
                        this->robot_feedback_.joint_feedback[i].joint_position;
                    this->robot_control_.joint_control[i].joint_positive_torque_limit =
                        robot_model_.params_ptr->joint_torque_limit[i];
                    this->robot_control_.joint_control[i].joint_negative_torque_limit =
                        robot_model_.params_ptr->joint_torque_limit[i];
                }
            }
            break;
        case STATUSWORD_MOTOR_ENABLED:
            this->is_robot_ready_ = true;
            inited = true;
            this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
            if (!this->is_cmd_valid_) {
                line_interpolator_.Reset();
                for (int i = 0; i < ndof_robot; i++) {
                    this->robot_control_.joint_control[i].joint_position =
                        this->robot_feedback_.joint_feedback[i].joint_position;
                    this->robot_control_.joint_control[i].joint_positive_torque_limit =
                        robot_model_.params_ptr->joint_torque_limit[i];
                    this->robot_control_.joint_control[i].joint_negative_torque_limit =
                        robot_model_.params_ptr->joint_torque_limit[i];
                }
            }
            break;
        case STATUSWORD_MOTOR_NOT_READY:
        case STATUSWORD_MOTOR_FAULT:
        case STATUSWORD_MOTOR_QUICK_STOP:
        default:
            line_interpolator_.Reset();
            for (int i = 0; i < ndof_robot; i++) {
                this->robot_control_.joint_control[i].joint_position =
                    this->robot_feedback_.joint_feedback[i].joint_position;
                this->robot_control_.joint_control[i].joint_positive_torque_limit =
                    robot_model_.params_ptr->joint_torque_limit[i];
                this->robot_control_.joint_control[i].joint_negative_torque_limit =
                    robot_model_.params_ptr->joint_torque_limit[i];
            }
            break;
    }
    // 2. 如果初次启动或者状态字是故障，则发送清除故障命令
    if (!inited || manager_info_.robot_property.clear_error_flag) {
        this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CLEAR_FAULT;
    }
}

void RobotServer::PublishThreadFunc() {
    if (!is_sim_) {
        Setup_Realtime(60);
    }
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(publish_dt_micro);
        this->robot_server_state_publisher_->publish(GetRobotServerStateMsg());
        auto now_time = std::chrono::steady_clock::now();
        if (now_time > next_cycle_time_point) {
            next_cycle_time_point = now_time;
        }
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

// clang-format off
std_msgs::msg::String RobotServer::GetRobotServerStateMsg()
{
    auto robot_server_state_msg = std_msgs::msg::String();
    uint64_t timestamp_ns = this->now().nanoseconds();
    // A1/Arm模式下数据在虚拟20关节表示中的6-19位置，需要提取
    int index_offset = GetFirstJointIndex();
    // 从 JointFeedback 数组中提取字段
    std::array<double, MAX_JOINT_NUM> position_arr_{}, velocity_arr_{}, torque_arr_{}, torque_sensor_arr_{}, temperature_arr_{};
    std::array<uint16_t, MAX_JOINT_NUM> motor_status_arr_{}, error_code_arr_{};
    for (int i = 0; i < ndof_robot; ++i) {
        position_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_position;
        velocity_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_velocity;
        torque_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_torque;
        torque_sensor_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_sensor_torque;
        motor_status_arr_[i] = this->robot_feedback_.al_status[i];
        temperature_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_temperature;
        error_code_arr_[i] = this->robot_feedback_.joint_feedback[i].joint_error_code;
    }
    // 从 JointControl 数组中提取字段
    std::array<double, MAX_JOINT_NUM> position_cmd_arr_{}, velocity_cmd_arr_{}, torque_ff_arr_{}, positive_torque_limit_arr_{}, negative_torque_limit_arr_{};
    for (int i = 0; i < ndof_robot; ++i) {
        position_cmd_arr_[i] = this->robot_control_.joint_control[i].joint_position;
        velocity_cmd_arr_[i] = this->robot_control_.joint_control[i].joint_velocity;
        torque_ff_arr_[i] = this->robot_control_.joint_control[i].joint_torque_feedforward;
        positive_torque_limit_arr_[i] = this->robot_control_.joint_control[i].joint_positive_torque_limit;
        negative_torque_limit_arr_[i] = this->robot_control_.joint_control[i].joint_negative_torque_limit;
    }

    robot_server_state_msg.data =
        std::string("{\n  \"timestamp\":") + std::to_string(timestamp_ns) +
        ", \"robot_type\":\"" + this->robot_model_.params_ptr->robot_type + "\"" +
        ", \"robot_version\":\"" + this->robot_model_.params_ptr->robot_version + "\"" +
        ", \"robot_mode\":\"" + ToString(this->robot_mode_) + "\"" +
        ",\n  \"status\":\"" + ToString(this->manager_state_) + "\"" +
        ", \"robot_statusword\":" + utils::RobotStatuswordToJsonString(this->robot_feedback_.robot_status_word).str() +
        ", \"robot_controlword\":" + utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str() +
        ",\n  \"motor_status\":" + utils::MotorStatusToJsonArrayString(motor_status_arr_, index_offset, ndof_config_).str() +
        ",\n  \"motor_error_code\":" + utils::VectorToJsonArrayString(error_code_arr_, index_offset, ndof_config_).str() +
        ",\n  \"motor_temperature\":" + utils::VectorToJsonArrayString(temperature_arr_, index_offset, ndof_config_).str() +
        ",\n  \"run_mode\":\"" + ToString(this->manager_info_.robot_property.run_mode) + "\"" +
        ", \"allow_force_control\":\"" + std::to_string(this->manager_info_.robot_property.allow_force_control) + "\"" +
        ", \"enable_cartesian_impedance\":\"" + std::to_string(this->manager_info_.robot_property.enable_cartesian_impedance) + "\"" +
        ", \"server_error_code\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.error_codes).str() +
        ", \"safe_level\":\"" + std::to_string(this->manager_info_.robot_property.safe_level) + "\"" +
        ", \"global_speed_ratio\":\"" + std::to_string(this->manager_info_.robot_property.global_speed_ratio) + "\"" +
        ",\n  \"joint_name\":" + utils::VectorToJsonArrayString(this->robot_model_.params_ptr->joint_names, index_offset, ndof_config_).str() +
        ",\n  \"joint_position\":" + utils::VectorToJsonArrayString(position_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_position_cmd\":" + utils::VectorToJsonArrayString(position_cmd_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_velocity\":" + utils::VectorToJsonArrayString(velocity_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_velocity_cmd\":" + utils::VectorToJsonArrayString(velocity_cmd_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_torque\":" + utils::VectorToJsonArrayString(torque_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_torque_cmd\":" + utils::VectorToJsonArrayString(torque_ff_arr_, index_offset, ndof_config_).str() +
        ",\n  \"joint_torque_sensor\":" + utils::VectorToJsonArrayString(torque_sensor_arr_, index_offset, ndof_config_).str() +
        ",\n  \"force_sensor\":" + utils::VectorToJsonArrayString(sensor_wrench_).str() +
        ",\n  \"raw_force_sensor\":" + utils::VectorToJsonArrayString(raw_sensor_wrench_).str() +
        ",\n  \"force_sensor_bias\":" + utils::VectorToJsonArrayString(sensor_zero_offset_).str() +
        ",\n  \"torque_limit_negative\":" + utils::VectorToJsonArrayString(negative_torque_limit_arr_, index_offset, ndof_config_).str() +
        ",\n  \"torque_limit_positive\":" + utils::VectorToJsonArrayString(positive_torque_limit_arr_, index_offset, ndof_config_).str() +
        ",\n  \"ee_pose\":" + utils::VectorToJsonArrayString(this->manager_info_.ee_pose_measurement, 0, this->robot_mode_ == RobotMode::SingleArm? 7 : 14).str() +
        ",\n  \"ee_pose_world\":" + utils::VectorToJsonArrayString(this->manager_info_.ee_pose_world_measurement, 0, this->robot_mode_ == RobotMode::SingleArm? 7 : 14).str() +
        ",\n  \"torso_pose_X_Z_RY\":" + utils::VectorToJsonArrayString(this->manager_info_.torso_pose_measurement).str() +
        ",\n  \"latest_queued_id\":" + std::to_string(this->latest_report_id_.queued) +
        ", \"latest_finished_id\":" + std::to_string(this->latest_report_id_.finished) +
        ", \"latest_errordrop_id\":" + std::to_string(this->latest_report_id_.error_drop) +
        ",\n  \"enable_streaming_ruckig\":" + std::to_string(this->manager_info_.robot_property.enable_streaming_ruckig) +
        ", \"joint_streaming_bounds\":" + this->manager_info_.robot_property.joint_streaming_bounds.ToString() +
        ",\n  \"enable_butterworth_filter\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.enable_butterworth_filter, index_offset, ndof_config_).str() +
        ",\n  \"butterworth_filter_frequency\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.butterworth_filter_frequency, index_offset, ndof_config_).str() +
        ",\n  \"enable_input_shaper\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.enable_input_shaper, index_offset, ndof_config_).str() +
        ",\n  \"input_shaper_frequency\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.input_shaper_frequency, index_offset, ndof_config_).str() +
        ",\n  \"enable_record\":" + std::to_string(this->manager_info_.robot_property.enable_record) +
        ", \"enable_chassis_shaper\":" + std::to_string(this->manager_info_.robot_property.enable_chassis_shaper) +
        ",\n  \"joint_stiffness\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_stiffness, index_offset, ndof_config_).str() +
        ",\n  \"joint_damping\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_damping, index_offset, ndof_config_).str() +
        ",\n  \"joint_integral_ratio\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.joint_integral_ratio, index_offset, ndof_config_).str() +
        ",\n  \"cart_stiffness\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_stiffness, 0, this->robot_mode_ == RobotMode::SingleArm? 6 : 12).str() +
        ",\n  \"cart_damping_ratio\":" + utils::VectorToJsonArrayString(this->manager_info_.robot_property.cart_damping_ratio, 0, this->robot_mode_ == RobotMode::SingleArm? 6 : 12).str() +
        ",\n  \"chassis_status_word\":" + std::to_string(this->robot_feedback_.chassis_feedback.status_word) +
        ", \"chassis_error_code_left\":" + std::to_string(this->robot_feedback_.chassis_feedback.error_code_left) +
        ", \"chassis_error_code_right\":" + std::to_string(this->robot_feedback_.chassis_feedback.error_code_right) +
        ",\n \"chassis_left_velocity\":" + std::to_string(this->robot_feedback_.chassis_feedback.left_velocity) +
        ", \"chassis_right_velocity\":" + std::to_string(this->robot_feedback_.chassis_feedback.right_velocity) +
        ", \"chassis_left_current\":" + std::to_string(this->robot_feedback_.chassis_feedback.left_current) +
        ", \"chassis_right_current\":" + std::to_string(this->robot_feedback_.chassis_feedback.right_current) +
        ",\n \"chassis_driver_temperature\":" + std::to_string(this->robot_feedback_.chassis_feedback.driver_temperature) +
        ", \"chassis_left_temperature\":" + std::to_string(this->robot_feedback_.chassis_feedback.left_temperature) +
        ", \"chassis_right_temperature\":" + std::to_string(this->robot_feedback_.chassis_feedback.right_temperature) +
        ",\n  \"tool_inertia_force_norm\":" + std::to_string(this->tool_inertia_force_norm_.load(std::memory_order_relaxed)) +
        ", \"tool_inertia_torque_norm\":" + std::to_string(this->tool_inertia_torque_norm_.load(std::memory_order_relaxed)) +
        ", \"tool_inertia_filter_cutoff_hz\":" + std::to_string(this->tool_inertia_filter_cutoff_hz_) +
        "}";
    return robot_server_state_msg;
}
// clang-format on

void RobotServer::PublishImu() {
    const auto& imu = robot_feedback_.imu_data;
    if (imu_timestamp_ != imu.timestamp) {
        imu_timestamp_ = imu.timestamp;
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_chassis_link";

        imu_msg.orientation.w = imu.orientation.w;
        imu_msg.orientation.x = imu.orientation.x;
        imu_msg.orientation.y = imu.orientation.y;
        imu_msg.orientation.z = imu.orientation.z;

        imu_msg.angular_velocity.x = imu.angular_velocity.x;
        imu_msg.angular_velocity.y = imu.angular_velocity.y;
        imu_msg.angular_velocity.z = imu.angular_velocity.z;

        imu_msg.linear_acceleration.x = imu.linear_acceleration.x;
        imu_msg.linear_acceleration.y = imu.linear_acceleration.y;
        imu_msg.linear_acceleration.z = imu.linear_acceleration.z;

        imu_publisher_->publish(imu_msg);
    }
}

void RobotServer::PublishFeedback() {
    // 发布关节状态反馈，A1模式只发布14个关节，W1模式发布20个关节
    sensor_msgs::msg::JointState feedback_msg;
    feedback_msg.header.stamp = this->get_clock()->now();

    feedback_msg.name.resize(ndof_config_);
    feedback_msg.position.resize(ndof_config_);
    feedback_msg.velocity.resize(ndof_config_);
    feedback_msg.effort.resize(ndof_config_);

    int index_offset = GetFirstJointIndex();
    for (int i = 0; i < ndof_config_; ++i) {
        feedback_msg.name[i] = robot_model_.params_ptr->joint_names[index_offset + i];
        feedback_msg.position[i] = robot_feedback_.joint_feedback[index_offset + i].joint_position;
        feedback_msg.velocity[i] = robot_feedback_.joint_feedback[index_offset + i].joint_velocity;
        feedback_msg.effort[i] = robot_feedback_.joint_feedback[index_offset + i].joint_torque;
    }

    feedback_publisher_->publish(feedback_msg);

    // 发布关节速度和力矩限制反馈
    joint_interfaces::msg::JointVelocityTorqueLimitFeedback limit_msg;
    limit_msg.header.stamp = this->get_clock()->now();

    limit_msg.name.resize(ndof_config_);
    limit_msg.positive_velocity_limit.resize(ndof_config_);
    limit_msg.negative_velocity_limit.resize(ndof_config_);
    limit_msg.positive_torque_limit.resize(ndof_config_);
    limit_msg.negative_torque_limit.resize(ndof_config_);

    // A1模式下控制数据在虚拟20关节表示中的6-19位置，需要提取
    for (int i = 0; i < ndof_config_; ++i) {
        limit_msg.name[i] = robot_model_.params_ptr->joint_names[index_offset + i];
        // 新结构体中没有 velocity_limit 字段，默认设置为0
        limit_msg.positive_velocity_limit[i] = 0.0;
        limit_msg.negative_velocity_limit[i] = 0.0;
        limit_msg.positive_torque_limit[i] =
            this->robot_control_.joint_control[index_offset + i].joint_positive_torque_limit;
        limit_msg.negative_torque_limit[i] =
            this->robot_control_.joint_control[index_offset + i].joint_negative_torque_limit;
    }

    velocity_torque_limit_feedback_publisher_->publish(limit_msg);
}

Int64 RobotServer::StartStreaming(PlannerRequest planner_request) {
    auto request_id = MakeNewRequestID(planner_request);

    if (request_id < 0) {
        return request_id;
    }
    planner_request.request_id = request_id;
    this->manager_buffers_.request_queue->EnQueue(planner_request);

    RCLCPP_INFO(this->get_logger(), "Request_id = %ld, StartJointStreaming Received", planner_request.request_id);
    LOG_INFO << "Request_id = " << planner_request.request_id << ", Received, type = StartJointStreaming";
    return request_id;
}

void RobotServer::ControlCallback(const JointControlMsg::SharedPtr msg) {
    auto robot_control_temp = robot_control_;
    bool position_valid = true;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_robot; ++j) {
            if (robot_model_.params_ptr->joint_names[j] == msg->name[i]) {
                // 新结构体中没有 velocity_limit 字段
                robot_control_.joint_control[j].joint_positive_torque_limit = msg->positive_torque_limit[i];
                robot_control_.joint_control[j].joint_negative_torque_limit = msg->negative_torque_limit[i];
                if (std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i]) >
                        position_diff_threshold_ &&
                    position_valid) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Control command for joint %s rejected: position difference %.4f exceeds threshold %.4f",
                        robot_model_.params_ptr->joint_names[j].c_str(),
                        std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i]),
                        position_diff_threshold_);
                    LOG_CRIT << "Control command for joint " << robot_model_.params_ptr->joint_names[j]
                             << " rejected: position difference "
                             << std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i])
                             << " exceeds threshold " << position_diff_threshold_;
                    position_valid = false;
                }
                robot_control_temp.joint_control[j].joint_position = msg->position[i];

                break;
            }
        }
    }
    // 仅当所有位置命令均有效时才启动或发送流式命令
    if (position_valid) {
        if (manager_state_ == Status::Idle) {
            PlannerRequest planner_request;
            planner_request.type = RequestType::StartJointStreaming;
            MoveRequest move_request;
            move_request.motion_group = MotionGroup::WholeBody;
            planner_request.data = move_request;
            streaming_request_id_ = StartStreaming(planner_request);
        }
        if (streaming_request_id_ > -1) {
            StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
            streaming_msg->request_id = streaming_request_id_;
            streaming_msg->streaming_finished = false;
            std::vector<float> position_vec(ndof_robot);
            for (int j = 0; j < ndof_robot; ++j) {
                position_vec[j] = robot_control_temp.joint_control[j].joint_position;
            }
            streaming_msg->position = position_vec;
            this->StreamingCommandCallback(streaming_msg);
        }
    }
}

void RobotServer::PositionControlCallback(const JointPositionControlMsg::SharedPtr msg) {
    auto robot_control_temp = robot_control_;
    bool position_valid = true;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_robot; ++j) {
            if (robot_model_.params_ptr->joint_names[j] == msg->name[i] && position_valid) {
                if (std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i]) >
                    position_diff_threshold_) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Control command for joint %s rejected: position difference %.4f exceeds threshold %.4f",
                        robot_model_.params_ptr->joint_names[j].c_str(),
                        std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i]),
                        position_diff_threshold_);
                    LOG_CRIT << "Control command for joint " << robot_model_.params_ptr->joint_names[j]
                             << " rejected: position difference "
                             << std::abs(robot_control_temp.joint_control[j].joint_position - msg->position[i])
                             << " exceeds threshold " << position_diff_threshold_;
                    position_valid = false;
                }
                robot_control_temp.joint_control[j].joint_position = msg->position[i];
                break;  // 已经找到并处理了对应的关节，跳出内层循环继续处理下一个输入关节
            } else if (robot_model_.params_ptr->ee_primary_name == msg->name[i]) {
                // 单独处理末端执行器位置命令
                break;
            } else if (robot_model_.params_ptr->ee_secondary_name == msg->name[i]) {
                // 单独处理末端执行器位置命令
                break;
            }
        }
    }
    // 所有关节位置均有效才发送
    if (position_valid) {
        if (manager_state_ == Status::Idle) {
            PlannerRequest planner_request;
            planner_request.type = RequestType::StartJointStreaming;
            MoveRequest move_request;
            move_request.motion_group = MotionGroup::WholeBody;
            move_request.streaming_timeout = streaming_timeout_;
            planner_request.data = move_request;
            streaming_request_id_ = StartStreaming(planner_request);
        }
        if (streaming_request_id_ != -1) {
            StreamingCommandMsg::SharedPtr streaming_msg = std::make_shared<StreamingCommandMsg>();
            streaming_msg->request_id = streaming_request_id_;
            streaming_msg->streaming_finished = false;
            std::vector<float> position_vec(ndof_robot);
            for (int j = 0; j < ndof_robot; ++j) {
                position_vec[j] = robot_control_temp.joint_control[j].joint_position;
            }
            streaming_msg->position = position_vec;
            this->StreamingCommandCallback(streaming_msg);
        }
    }
}

void RobotServer::DebugCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "/control/debug message received: %s", msg->data.c_str());
    LOG_INFO << "/control/debug message received: " << msg->data;

    try {
        const auto json_data = nlohmann::json::parse(msg->data);

        auto get_bool_value = [](const nlohmann::json& obj, const char* key) -> std::optional<bool> {
            if (!obj.contains(key)) return std::nullopt;
            const auto& v = obj[key];
            if (v.is_boolean()) return v.get<bool>();
            if (v.is_number_integer()) return v.get<int>() != 0;
            return std::nullopt;
        };

        auto update_bool_param = [this, &json_data, &get_bool_value](const char* key, bool& param) {
            auto value = get_bool_value(json_data, key);
            if (value.has_value()) {
                param = value.value();
                RCLCPP_INFO(this->get_logger(), "%s set to %s", key, param ? "true" : "false");
                LOG_INFO << key << " set to " << (param ? "true" : "false");
            }
        };

        auto update_atomic_bool_param = [this, &json_data, &get_bool_value](const char* key, std::atomic<bool>& param) {
            auto value = get_bool_value(json_data, key);
            if (value.has_value()) {
                bool bool_value = value.value();
                param.store(bool_value);
                RCLCPP_INFO(this->get_logger(), "%s set to %s", key, bool_value ? "true" : "false");
                LOG_INFO << key << " set to " << (bool_value ? "true" : "false");
            }
        };

    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse /control/debug JSON: %s", e.what());
    }
}

void RobotServer::OmniChassisControlCallback(const ChassisWholeControlMsg::SharedPtr msg) {
    chassis_control_target_.chassis_mode = 0;  // 设置为全向整车模式
    chassis_control_target_.chassis_whole_mode = msg->mode;
    chassis_control_target_.chassis_velocity = msg->speed;
    chassis_control_target_.chassis_direction = msg->direction;
    RCLCPP_DEBUG(this->get_logger(), "Omni chassis control: mode=%ld, speed=%.2f, direction=%.2f", msg->mode,
                 msg->speed, msg->direction);
}

// 全向底盘电机控制回调
void RobotServer::OmniChassisMotorControlCallback(const ChassisMotorControlMsg::SharedPtr msg) {
    chassis_control_target_.chassis_mode = 2;  // 设置为电机控制模式
    for (size_t i = 0; i < msg->speed.size() && i < OMNI_MOTOR_NUM; i++) {
        chassis_control_target_.chassis_motor_velocity[i] = msg->speed[i];
        chassis_control_target_.chassis_motor_direction[i] = msg->position[i];
    }
    chassis_control_target_.chassis_estop = msg->estop;
    chassis_control_target_.chassis_neutral = msg->neutral;
    RCLCPP_DEBUG(this->get_logger(), "Omni chassis motor control: estop=%ld, neutral=%ld", msg->estop, msg->neutral);
}

void RobotServer::DiffChassisControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    chassis_control_target_.chassis_mode = 1;  // 设置为差速模式
    double chassis_right_velocity = msg->linear.x + msg->angular.z * DiffChassisWidth * 0.5;
    double chassis_left_velocity = msg->linear.x - msg->angular.z * DiffChassisWidth * 0.5;

    chassis_control_target_.chassis_right_velocity = chassis_right_velocity * 60 / (M_PI * DiffChassisWheelD);
    chassis_control_target_.chassis_left_velocity = chassis_left_velocity * 60 / (M_PI * DiffChassisWheelD);

    RCLCPP_DEBUG(this->get_logger(), "Diff chassis control: linear.x=%.2f, angular.z=%.2f => left=%.2f, right=%.2f",
                 msg->linear.x, msg->angular.z, chassis_control_target_.chassis_left_velocity,
                 chassis_control_target_.chassis_right_velocity);
}

bool RobotServer::SortFeedbackByJointName(RobotFeedback& feedback) {
    // 检测反馈数据中的有效关节数量
    int valid_count = 0;
    for (int i = 0; i < MAX_JOINT_NUM; ++i) {
        if (strlen(feedback.joint_feedback[i].joint_name) > 0) {
            valid_count++;
        }
    }

    if (valid_count != ndof_config_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "SortFeedbackByJointName: joint name count mismatch. Expected %d, got %d", ndof_config_,
                              valid_count);
        LOG_CRIT_THROTTLE(5000) << "SortFeedbackByJointName: joint name count mismatch. Expected " << ndof_config_
                                << ", got " << valid_count;
        return false;
    }

    // 创建临时数组，初始化为默认值
    std::array<JointFeedback, MAX_JOINT_NUM> sorted_joints{};
    int matched_count = 0;

    for (int target_idx = 0; target_idx < ndof_robot; ++target_idx) {
        const std::string& target_name = robot_model_.params_ptr->joint_names[target_idx];

        // 在反馈数据中查找匹配的关节名称
        for (int src_idx = 0; src_idx < MAX_JOINT_NUM; ++src_idx) {
            std::string src_name(feedback.joint_feedback[src_idx].joint_name);
            if (src_name == target_name) {
                // 找到匹配，复制数据到目标位置
                sorted_joints[target_idx] = feedback.joint_feedback[src_idx];
                matched_count++;
                break;
            }
        }
    }

    if (matched_count != ndof_config_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "SortFeedbackByJointName: matched count mismatch. Expected %d, matched %d", ndof_config_,
                              matched_count);
        LOG_CRIT_THROTTLE(5000) << "SortFeedbackByJointName: matched count mismatch. Expected " << ndof_config_
                                << ", matched " << matched_count;
        return false;
    }

    // 只有在成功匹配时才重置日志标志
    has_logged_sort_feedback_error_ = false;

    // 将排序后的数据复制回反馈结构
    for (int i = 0; i < ndof_robot; ++i) {
        feedback.joint_feedback[i] = sorted_joints[i];
    }

    return true;
}

void RobotServer::SortControlByJointName(RobotControl& control) {
    const auto& joint_names_config = robot_model_.params_ptr->joint_names;
    const std::vector<std::string>& mode_joint_names = [this]() -> const std::vector<std::string>& {
        switch (robot_mode_) {
            case RobotMode::A1:
                return robot_model_.params_ptr->joints_a1;
            case RobotMode::SingleArm:
                return robot_model_.params_ptr->joints_single_arm;
            case RobotMode::DualArm:
                return robot_model_.params_ptr->joints_dual_arm;
            default:
                return robot_model_.params_ptr->joint_names;
        }
    }();

    for (int i = 0; i < ndof_config_; ++i) {
        const std::string& target_name = mode_joint_names[i];
        for (int j = 0; j < ndof_robot; ++j) {
            if (joint_names_config[j] == target_name) {
                control.joint_control[i] = robot_control_.joint_control[j];
                strncpy(control.joint_control[i].joint_name, target_name.c_str(),
                        sizeof(control.joint_control[i].joint_name) - 1);
                control.joint_control[i].joint_name[sizeof(control.joint_control[i].joint_name) - 1] = '\0';
                break;
            }
        }
    }
}

RobotMode RobotServer::DetectRobotModeFromFeedback(const RobotFeedback& feedback) {
    std::set<std::string> feedback_joint_names;
    for (int i = 0; i < ndof_robot; ++i) {
        std::string jname(feedback.joint_feedback[i].joint_name);
        if (!jname.empty()) {
            feedback_joint_names.insert(jname);
        }
    }

    auto is_exact_match = [&feedback_joint_names](const std::vector<std::string>& config_joints) -> bool {
        if (feedback_joint_names.size() != config_joints.size()) {
            return false;
        }
        for (const auto& j : config_joints) {
            if (!feedback_joint_names.count(j)) {
                return false;
            }
        }
        return true;
    };

    if (is_exact_match(robot_model_.params_ptr->joint_names)) {
        robot_mode_ = RobotMode::W1;
        ndof_config_ = ndof_robot;
        LOG_INFO << "自动检测到 W1 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 W1 模式，实际关节数: %d", ndof_config_);
        has_logged_unknown_mode_ = false;
        return RobotMode::W1;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_dual_arm)) {
        robot_mode_ = RobotMode::DualArm;
        ndof_config_ = robot_model_.params_ptr->joints_dual_arm.size();
        LOG_INFO << "自动检测到 DualArm 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 DualArm 模式，实际关节数: %d", ndof_config_);
        has_logged_unknown_mode_ = false;
        return RobotMode::DualArm;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_a1)) {
        robot_mode_ = RobotMode::A1;
        ndof_config_ = robot_model_.params_ptr->joints_a1.size();
        LOG_INFO << "自动检测到 A1 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 A1 模式，实际关节数: %d", ndof_config_);
        has_logged_unknown_mode_ = false;
        return RobotMode::A1;
    }
    if (is_exact_match(robot_model_.params_ptr->joints_single_arm)) {
        robot_mode_ = RobotMode::SingleArm;
        ndof_config_ = robot_model_.params_ptr->joints_single_arm.size();
        LOG_INFO << "自动检测到 SingleArm 模式，实际关节数: " << ndof_config_;
        RCLCPP_INFO(this->get_logger(), "自动检测到 SingleArm 模式，实际关节数: %d", ndof_config_);
        has_logged_unknown_mode_ = false;
        return RobotMode::SingleArm;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "未匹配到任何已知模式");
    LOG_WARN_THROTTLE(5000) << "未匹配到任何已知模式";
    return RobotMode::Unknown;
}

bool RobotServer::IsArmJoint(int joint_index) const {
    if (joint_index < 0 || joint_index >= ndof_robot) {
        return false;
    }
    const std::string& joint_name = robot_model_.params_ptr->joint_names[joint_index];

    for (const auto& name : robot_model_.params_ptr->joints_dual_arm) {
        if (joint_name == name) return true;
    }
    return false;
}

int RobotServer::GetFirstJointIndex() const {
    auto mode_joint_names = robot_model_.params_ptr->joint_names;
    switch (robot_mode_) {
        case RobotMode::SingleArm:
            mode_joint_names = robot_model_.params_ptr->joints_single_arm;
            break;
        case RobotMode::DualArm:
            mode_joint_names = robot_model_.params_ptr->joints_dual_arm;
            break;
        case RobotMode::A1:
            mode_joint_names = robot_model_.params_ptr->joints_a1;
            break;
        case RobotMode::W1:
        default:
            mode_joint_names = robot_model_.params_ptr->joint_names;
            break;
    }

    const std::string& first_joint_name = mode_joint_names[0];
    for (int i = 0; i < ndof_robot; ++i) {
        if (robot_model_.params_ptr->joint_names[i] == first_joint_name) {
            return i;
        }
    }
    return 0;
}

}  // namespace robot_server_lib
