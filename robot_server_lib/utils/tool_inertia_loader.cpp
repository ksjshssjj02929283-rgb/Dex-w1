#include "tool_compensation.h"

#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace robot_server_lib {
namespace dynamics {
namespace tool_compensation {

namespace {
void CheckToolInertia(const pinocchio::Inertia& inertia, const std::filesystem::path& urdf_path);
}

std::filesystem::path ResolveToolUrdfPath(const std::string& robot_version,
                                         const std::string& hand_type,
                                         const std::string& side) {
    return std::filesystem::path(std::getenv("HOME")) / ".cache/dexe_data/extracted/end_effectors" /
           (robot_version.substr(0, 3) + "x") / hand_type / (side + "_hand") / (side + "_hand.urdf");
}

pinocchio::Inertia LoadToolInertia(const std::filesystem::path& urdf_path) {
    pinocchio::Model hand_model;
    pinocchio::urdf::buildModel(urdf_path.string(), hand_model, false, true);
    std::vector<pinocchio::JointIndex> locked_joints;
    locked_joints.reserve(hand_model.njoints - 1);
    for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(hand_model.njoints); ++i) {
        locked_joints.push_back(i);
    }
    const auto reduced_model = pinocchio::buildReducedModel(hand_model, locked_joints, pinocchio::neutral(hand_model));
    const auto inertia = reduced_model.inertias[0];
    CheckToolInertia(inertia, urdf_path);
    return inertia;
}

namespace {
void CheckToolInertia(const pinocchio::Inertia& inertia, const std::filesystem::path& urdf_path) {
    if (!std::isfinite(inertia.mass()) || inertia.mass() <= 0.0 || !inertia.lever().allFinite() ||
        !inertia.inertia().matrix().allFinite()) {
        throw std::runtime_error("Invalid tool inertia in URDF: " + urdf_path.string());
    }
}
}  // namespace

}  // namespace tool_compensation
}  // namespace dynamics
}  // namespace robot_server_lib
