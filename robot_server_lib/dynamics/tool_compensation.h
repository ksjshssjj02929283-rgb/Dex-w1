#pragma once

#include <filesystem>

#include "pinocchio_api.h"
#include "typedef.h"

namespace robot_server_lib {
namespace dynamics {
namespace tool_compensation {

struct ToolDynamicsConfig {
    bool enabled = false;
    pinocchio::Inertia inertia = pinocchio::Inertia::Zero();
};

pinocchio::Inertia LoadToolInertia(const std::filesystem::path& urdf_path);

class ToolCompensation {
public:
    ToolCompensation(const pinocchio::Model& arm_model, const pinocchio::Inertia& tool_inertia);

    Eigen::VectorXd ComputeTorque(const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& dq,
                                  const Eigen::VectorXd& ddq,
                                  const Vector3d& gravity_armbase);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    Matrix3d root_R_armbase_ = Matrix3d::Identity();
};

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
