#pragma once

#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "butterworth_filter.h"
#include "data_structure.h"
#include "pinocchio_api.h"

namespace robot_server_lib {
namespace dynamics {
namespace tool_compensation {

constexpr double kDefaultGravity = 9.8;
constexpr double kDefaultInertiaCompCutoffHz = 5.0;
constexpr double kMinInertiaCompCutoffHz = 0.5;
constexpr double kMaxInertiaCompCutoffHz = 100.0;

struct ToolCompensationConfig {
    bool gravity_enabled = false;
    double mass_kg = 0.0;
    double gravity_magnitude = kDefaultGravity;
    int model_sign = 1;
    Vector3d com_m = Vector3d::Zero();
    bool inertia_enabled = false;
    Eigen::Matrix3d inertia_com = Eigen::Matrix3d::Zero();
    double inertia_filter_cutoff_hz = kDefaultInertiaCompCutoffHz;
};

struct ToolCompensationLoadResult {
    ToolCompensationConfig config{};
    bool config_present = false;
    bool config_valid = false;
    bool inertia_matrix_present = false;
    bool inertia_matrix_valid = false;
};

struct ToolCompensationState {
    bool inertia_first_sample = true;
    Vector3d prev_ee_linvel_armbase = Vector3d::Zero();
    Vector3d prev_omega_armbase = Vector3d::Zero();
    std::array<std::shared_ptr<ButterworthFilter>, 3> ee_accel_filter{};
    std::array<std::shared_ptr<ButterworthFilter>, 3> omega_diff_filter{};
    std::array<std::shared_ptr<ButterworthFilter>, 6> force_sensor_gravity_filter_primary{};
    std::array<std::atomic<double>, 6> tool_inertia_wrench_ee{};
    std::array<std::atomic<double>, 3> ee_omega_F_primary{};
    std::array<std::atomic<double>, 3> ee_alpha_F_primary{};
};

double JsonFiniteOrZero(double value);

bool ReadYamlVector(const YAML::Node& node, int expected_size, Eigen::VectorXd& out);
bool ReadYamlMatrix3(const YAML::Node& node, Eigen::Matrix3d& out);

ToolCompensationLoadResult LoadToolGravityCompensationConfig(const std::filesystem::path& robot_cfg_path,
                                                             std::vector<std::string>* warnings = nullptr);

void InitializeToolCompensationFilters(ToolCompensationState& state, double cutoff_hz);

Vector6d ComputeToolGravityWrenchEE(const ToolCompensationConfig& config,
                                    const std::optional<Eigen::Matrix3d>& rotation_world_from_ee,
                                    bool* applied = nullptr,
                                    std::string* failure_reason = nullptr);

Vector6d ApplyToolGravityCompensation(const Vector6d& wrench_ee,
                                      const ToolCompensationConfig& config,
                                      const std::optional<Eigen::Matrix3d>& rotation_world_from_ee,
                                      bool* applied = nullptr,
                                      std::string* failure_reason = nullptr);

Vector6d ApplyToolGravityMatchedFilterPrimary(const Vector6d& wrench_ee, ToolCompensationState& state);

Vector6d ApplyToolInertiaCompensationPrimary(const Vector6d& wrench_ee,
                                             const pinocchio::Model& arm_model,
                                             const Eigen::VectorXd& feedback_q_primary,
                                             const Eigen::VectorXd& feedback_dq_primary,
                                             const ToolCompensationConfig& config,
                                             ToolCompensationState& state,
                                             bool* applied = nullptr,
                                             std::string* failure_reason = nullptr);

Eigen::VectorXd ComputeToolJointTorqueFromWrench(const pinocchio::Model& arm_model,
                                                 const Eigen::VectorXd& feedback_q_arm,
                                                 const Vector6d& tool_wrench_ee,
                                                 bool* applied = nullptr,
                                                 std::string* failure_reason = nullptr);

void ResetToolInertiaCompensationState(ToolCompensationState& state);

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
