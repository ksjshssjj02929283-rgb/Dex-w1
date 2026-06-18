#include "tool_compensation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "utils.h"

namespace robot_server_lib {
namespace dynamics {
namespace tool_compensation {

double JsonFiniteOrZero(double value) { return std::isfinite(value) ? value : 0.0; }

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

ToolCompensationLoadResult LoadToolGravityCompensationConfig(const std::filesystem::path& robot_cfg_path,
                                                             std::vector<std::string>* warnings) {
    ToolCompensationLoadResult result;

    try {
        const auto config = YAML::LoadFile(robot_cfg_path.string());
        const auto node = config["tool_gravity_compensation"];
        if (!node) {
            return result;
        }

        result.config_present = true;
        Eigen::VectorXd com_mm;
        if (!ReadYamlVector(node["com_mm"], 3, com_mm) || !node["mass_kg"]) {
            if (warnings) warnings->push_back("Invalid tool_gravity_compensation config, disabled");
            return result;
        }

        result.config.mass_kg = node["mass_kg"].as<double>();
        if (node["gravity_magnitude"]) {
            result.config.gravity_magnitude = node["gravity_magnitude"].as<double>();
        }
        if (node["model_sign"]) {
            result.config.model_sign = node["model_sign"].as<int>();
        }
        result.config.com_m = com_mm.head<3>() / 1000.0;
        result.config.gravity_enabled = true;
        result.config_valid = true;

        result.inertia_matrix_present = static_cast<bool>(node["inertia_com_kg_m2"]);
        Eigen::Matrix3d inertia_com;
        if (ReadYamlMatrix3(node["inertia_com_kg_m2"], inertia_com)) {
            result.inertia_matrix_valid = true;
            if (result.config.mass_kg < FloatZeroTol) {
                if (warnings) {
                    warnings->push_back(
                        "tool inertia compensation disabled: mass_kg is ~0 while inertia_com_kg_m2 is set");
                }
            } else {
                result.config.inertia_com = inertia_com;
                result.config.inertia_enabled = true;
            }
        } else if (result.inertia_matrix_present) {
            if (warnings)
                warnings->push_back("Invalid inertia_com_kg_m2 (expect 3x3 list), tool inertia compensation disabled");
        }

        if (node["inertia_compensation_filter_hz"]) {
            const double v = node["inertia_compensation_filter_hz"].as<double>();
            if (!std::isfinite(v) || v < kMinInertiaCompCutoffHz || v > kMaxInertiaCompCutoffHz) {
                if (warnings) warnings->push_back("inertia_compensation_filter_hz out of range, using default");
            } else {
                result.config.inertia_filter_cutoff_hz = v;
            }
        }
    } catch (const YAML::Exception& e) {
        if (warnings) warnings->push_back(std::string("Failed to load tool gravity compensation config: ") + e.what());
    }

    return result;
}

void InitializeToolCompensationFilters(ToolCompensationState& state, double cutoff_hz) {
    for (auto& f : state.ee_accel_filter) {
        f = std::make_shared<ButterworthFilter>(cutoff_hz);
    }
    for (auto& f : state.omega_diff_filter) {
        f = std::make_shared<ButterworthFilter>(cutoff_hz);
    }
    for (auto& f : state.force_sensor_gravity_filter_primary) {
        f = std::make_shared<ButterworthFilter>(cutoff_hz);
    }
}

Vector6d ComputeToolGravityWrenchEE(const ToolCompensationConfig& config,
                                    const std::optional<Eigen::Matrix3d>& rotation_world_from_ee,
                                    bool* applied,
                                    std::string* failure_reason) {
    if (applied) *applied = false;
    if (failure_reason) failure_reason->clear();
    if (!config.gravity_enabled) {
        return Vector6d::Zero();
    }
    if (!rotation_world_from_ee) {
        if (failure_reason) *failure_reason = "missing valid ee_pose_world rotation";
        return Vector6d::Zero();
    }

    const Vector3d gravity_world(0.0, 0.0, -config.gravity_magnitude);
    const Vector3d expected_force_ee =
        static_cast<double>(config.model_sign) * rotation_world_from_ee->transpose() * (config.mass_kg * gravity_world);

    Vector6d expected_wrench_ee;
    expected_wrench_ee.head<3>() = expected_force_ee;
    expected_wrench_ee.tail<3>() = config.com_m.cross(expected_force_ee);

    if (!utils::IsFiniteVector(expected_wrench_ee)) {
        if (failure_reason) *failure_reason = "expected wrench is non-finite";
        return Vector6d::Zero();
    }

    if (applied) *applied = true;
    return expected_wrench_ee;
}

Vector6d ApplyToolGravityCompensation(const Vector6d& wrench_ee,
                                      const ToolCompensationConfig& config,
                                      const std::optional<Eigen::Matrix3d>& rotation_world_from_ee,
                                      bool* applied,
                                      std::string* failure_reason) {
    const Vector6d expected_wrench_ee =
        ComputeToolGravityWrenchEE(config, rotation_world_from_ee, applied, failure_reason);
    if (applied && !*applied) {
        return wrench_ee;
    }
    return wrench_ee - expected_wrench_ee;
}

Vector6d ApplyToolGravityMatchedFilterPrimary(const Vector6d& wrench_ee, ToolCompensationState& state) {
    Vector6d filtered = wrench_ee;
    for (int i = 0; i < 6; ++i) {
        if (state.force_sensor_gravity_filter_primary[i]) {
            filtered[i] = state.force_sensor_gravity_filter_primary[i]->Apply(wrench_ee[i]);
        }
    }
    return filtered;
}

Vector6d ApplyToolInertiaCompensationPrimary(const Vector6d& wrench_ee,
                                             const pinocchio::Model& arm_model,
                                             const Eigen::VectorXd& feedback_q_primary,
                                             const Eigen::VectorXd& feedback_dq_primary,
                                             const ToolCompensationConfig& config,
                                             ToolCompensationState& state,
                                             bool* applied,
                                             std::string* failure_reason) {
    if (applied) *applied = false;
    if (failure_reason) failure_reason->clear();

    auto fail_inertia_compensation = [&](const char* reason) {
        ResetToolInertiaCompensationState(state);
        if (failure_reason) *failure_reason = reason;
        return wrench_ee;
    };

    if (!config.inertia_enabled) {
        ResetToolInertiaCompensationState(state);
        return wrench_ee;
    }
    if (!config.gravity_enabled) {
        return fail_inertia_compensation("gravity compensation is disabled");
    }
    if (arm_model.nv != 7 || feedback_q_primary.size() != 7 || feedback_dq_primary.size() != 7) {
        return fail_inertia_compensation("primary arm model or feedback vector size is invalid");
    }
    if (!utils::IsFiniteVector(wrench_ee) || !utils::IsFiniteVector(feedback_q_primary) ||
        !utils::IsFiniteVector(feedback_dq_primary)) {
        return fail_inertia_compensation("input wrench/q/dq contains non-finite values");
    }

    constexpr double kDt = torque_ctrl_dt;

    const Eigen::MatrixXd J_armbase_dyn = pinocchio::GetJacobiMatrix(arm_model, feedback_q_primary);
    if (J_armbase_dyn.rows() != 6 || J_armbase_dyn.cols() != 7) {
        return fail_inertia_compensation("Jacobian shape is invalid");
    }
    if (!utils::IsFiniteVector(J_armbase_dyn)) {
        return fail_inertia_compensation("Jacobian contains non-finite values");
    }
    const Eigen::Matrix<double, 6, 7> J_armbase = J_armbase_dyn;
    const Eigen::Matrix<double, 7, 1> dq7 = feedback_dq_primary.head<7>();
    const Eigen::Matrix<double, 6, 1> twist_armbase = J_armbase * dq7;
    if (!utils::IsFiniteVector(twist_armbase)) {
        return fail_inertia_compensation("end-effector twist contains non-finite values");
    }
    const Vector3d v_ee_armbase = twist_armbase.head<3>();
    const Vector3d w_ee_armbase = twist_armbase.tail<3>();

    if (state.inertia_first_sample) {
        for (int i = 0; i < 3; ++i) {
            if (!state.ee_accel_filter[i] || !state.omega_diff_filter[i]) {
                return fail_inertia_compensation("inertia compensation filters are not initialized");
            }
        }
        for (int i = 0; i < 3; ++i) state.ee_accel_filter[i]->Apply(0.0);
        for (int i = 0; i < 3; ++i) state.omega_diff_filter[i]->Apply(0.0);
        state.prev_ee_linvel_armbase = v_ee_armbase;
        state.prev_omega_armbase = w_ee_armbase;
        state.inertia_first_sample = false;
        for (int i = 0; i < 6; ++i) {
            state.tool_inertia_wrench_ee[i].store(0.0, std::memory_order_relaxed);
        }
        const double nan_v = std::numeric_limits<double>::quiet_NaN();
        for (int i = 0; i < 3; ++i) {
            state.ee_omega_F_primary[i].store(nan_v, std::memory_order_relaxed);
            state.ee_alpha_F_primary[i].store(nan_v, std::memory_order_relaxed);
        }
        return wrench_ee;
    }

    const Vector3d a_raw_armbase = (v_ee_armbase - state.prev_ee_linvel_armbase) / kDt;
    Vector3d a_ee_armbase;
    for (int i = 0; i < 3; ++i) {
        if (!state.ee_accel_filter[i]) {
            return fail_inertia_compensation("linear acceleration filter is not initialized");
        }
        a_ee_armbase[i] = state.ee_accel_filter[i]->Apply(a_raw_armbase[i]);
    }

    const Vector3d alpha_raw = (w_ee_armbase - state.prev_omega_armbase) / kDt;
    Vector3d alpha_ee_armbase;
    for (int i = 0; i < 3; ++i) {
        if (!state.omega_diff_filter[i]) {
            return fail_inertia_compensation("angular acceleration filter is not initialized");
        }
        alpha_ee_armbase[i] = state.omega_diff_filter[i]->Apply(alpha_raw[i]);
    }
    if (!utils::IsFiniteVector(a_ee_armbase) || !utils::IsFiniteVector(alpha_ee_armbase)) {
        return fail_inertia_compensation("filtered acceleration contains non-finite values");
    }
    state.prev_ee_linvel_armbase = v_ee_armbase;
    state.prev_omega_armbase = w_ee_armbase;

    const Eigen::Matrix4d T_armbase_ee = pinocchio::GetFK(arm_model, feedback_q_primary);
    const Eigen::Matrix3d R_a_e = T_armbase_ee.topLeftCorner<3, 3>();
    if (!utils::IsFiniteVector(T_armbase_ee) || !utils::IsFiniteVector(R_a_e)) {
        return fail_inertia_compensation("end-effector FK contains non-finite values");
    }

    const Vector3d r_C_armbase = R_a_e * config.com_m;
    const Vector3d a_C_armbase =
        a_ee_armbase + alpha_ee_armbase.cross(r_C_armbase) + w_ee_armbase.cross(w_ee_armbase.cross(r_C_armbase));
    const Eigen::Matrix3d I_C_armbase = R_a_e * config.inertia_com * R_a_e.transpose();

    const Vector3d F_armbase = config.mass_kg * a_C_armbase;
    const Vector3d tau_about_ee_armbase =
        I_C_armbase * alpha_ee_armbase + w_ee_armbase.cross(I_C_armbase * w_ee_armbase) + r_C_armbase.cross(F_armbase);

    Vector6d expected_inertia_wrench_ee;
    expected_inertia_wrench_ee.head<3>() = -static_cast<double>(config.model_sign) * (R_a_e.transpose() * F_armbase);
    expected_inertia_wrench_ee.tail<3>() =
        -static_cast<double>(config.model_sign) * (R_a_e.transpose() * tau_about_ee_armbase);
    if (!utils::IsFiniteVector(expected_inertia_wrench_ee)) {
        return fail_inertia_compensation("expected inertia wrench contains non-finite values");
    }

    for (int i = 0; i < 6; ++i) {
        state.tool_inertia_wrench_ee[i].store(expected_inertia_wrench_ee[i], std::memory_order_relaxed);
    }

    const Vector3d w_ee_F = R_a_e.transpose() * w_ee_armbase;
    const Vector3d alpha_ee_F = R_a_e.transpose() * alpha_ee_armbase;
    if (!utils::IsFiniteVector(w_ee_F) || !utils::IsFiniteVector(alpha_ee_F)) {
        return fail_inertia_compensation("ee omega/alpha in EE frame contains non-finite values");
    }
    for (int i = 0; i < 3; ++i) {
        state.ee_omega_F_primary[i].store(w_ee_F[i], std::memory_order_relaxed);
        state.ee_alpha_F_primary[i].store(alpha_ee_F[i], std::memory_order_relaxed);
    }

    if (applied) *applied = true;
    return wrench_ee - expected_inertia_wrench_ee;
}

Eigen::VectorXd ComputeToolJointTorqueFromWrench(const pinocchio::Model& arm_model,
                                                 const Eigen::VectorXd& feedback_q_arm,
                                                 const Vector6d& tool_wrench_ee,
                                                 bool* applied,
                                                 std::string* failure_reason) {
    if (applied) *applied = false;
    if (failure_reason) failure_reason->clear();

    if (arm_model.nv <= 0 || feedback_q_arm.size() != arm_model.nq) {
        if (failure_reason) *failure_reason = "arm model or feedback q size is invalid";
        return Eigen::VectorXd::Zero(std::max<int>(arm_model.nv, 0));
    }
    if (!utils::IsFiniteVector(feedback_q_arm) || !utils::IsFiniteVector(tool_wrench_ee)) {
        if (failure_reason) *failure_reason = "tool torque input contains non-finite values";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    const Eigen::MatrixXd jacobian = pinocchio::GetJacobiMatrix(arm_model, feedback_q_arm, pinocchio::LOCAL);
    if (jacobian.rows() != 6 || jacobian.cols() != arm_model.nv) {
        if (failure_reason) *failure_reason = "Jacobian shape is invalid";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }
    if (!utils::IsFiniteVector(jacobian)) {
        if (failure_reason) *failure_reason = "Jacobian contains non-finite values";
        return Eigen::VectorXd::Zero(arm_model.nv);
    }

    const Eigen::VectorXd tau_force = jacobian.topRows<3>().transpose() * tool_wrench_ee.head<3>();
    const Eigen::VectorXd tau_moment = jacobian.bottomRows<3>().transpose() * tool_wrench_ee.tail<3>();
    const Eigen::VectorXd tau_tool = tau_force + tau_moment;

    static int print_count = 0;
    if (++print_count % 1000 == 0) {
        const Eigen::IOFormat log_format(5, 0, ", ", "\n", "[", "]");
        std::ostringstream ss;
        ss << "tool joint torque debug: wrench_ee=" << tool_wrench_ee.transpose().format(log_format)
           << ", q_arm=" << feedback_q_arm.transpose().format(log_format)
           << ", tau_force=" << tau_force.transpose().format(log_format)
           << ", tau_moment=" << tau_moment.transpose().format(log_format)
           << ", tau_tool=" << tau_tool.transpose().format(log_format);
        LOG_INFO << ss.str();
    }

    if (applied) *applied = true;
    return tau_tool;
}

void ResetToolInertiaCompensationState(ToolCompensationState& state) {
    state.inertia_first_sample = true;
    state.prev_ee_linvel_armbase.setZero();
    state.prev_omega_armbase.setZero();
    for (auto& f : state.ee_accel_filter) {
        if (f) f->Reset();
    }
    for (auto& f : state.omega_diff_filter) {
        if (f) f->Reset();
    }
    for (auto& f : state.force_sensor_gravity_filter_primary) {
        if (f) f->Reset();
    }
    for (int i = 0; i < 6; ++i) {
        state.tool_inertia_wrench_ee[i].store(0.0, std::memory_order_relaxed);
    }
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 3; ++i) {
        state.ee_omega_F_primary[i].store(nan_v, std::memory_order_relaxed);
        state.ee_alpha_F_primary[i].store(nan_v, std::memory_order_relaxed);
    }
}

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
