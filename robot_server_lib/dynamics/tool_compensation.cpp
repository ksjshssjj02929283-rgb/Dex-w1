#include "tool_compensation.h"

namespace robot_server_lib {
namespace dynamics {
namespace tool_compensation {

ToolCompensation::ToolCompensation(const pinocchio::Model& arm_model, const pinocchio::Inertia& tool_inertia)
    : model_(arm_model), data_(model_) {
    pinocchio::framesForwardKinematics(model_, data_, pinocchio::neutral(model_));
    const std::string suffix = "arm_base";
    for (pinocchio::FrameIndex i = 0; i < static_cast<pinocchio::FrameIndex>(model_.nframes); ++i) {
        const auto& name = model_.frames[i].name;
        if (name.size() >= suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
            root_R_armbase_ = data_.oMf[i].rotation();
            break;
        }
    }

    for (auto& inertia : model_.inertias) {
        inertia.setZero();
    }
    model_.armature.setZero();
    const auto& ee_frame = model_.frames.back();
    model_.appendBodyToJoint(ee_frame.parentJoint, tool_inertia, ee_frame.placement);
}

Eigen::VectorXd ToolCompensation::ComputeTorque(const Eigen::VectorXd& q,
                                               const Eigen::VectorXd& dq,
                                               const Eigen::VectorXd& ddq,
                                               const Vector3d& gravity_armbase) {
    // 独立臂 URDF 的根与 arm_base 之间包含固定旋转。
    model_.gravity.linear() = root_R_armbase_ * gravity_armbase;
    model_.gravity.angular().setZero();
    return pinocchio::rnea(model_, data_, q, dq, ddq);
}

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
