#include <gtest/gtest.h>

#include <filesystem>

#include "joint_impedance_controller.h"
#include "utils.h"

using namespace robot_server_lib;

namespace {

RobotModel LoadRobotModel() {
    std::filesystem::path home_dir = std::filesystem::path(std::getenv("HOME"));
    std::filesystem::create_directories(home_dir / "log");
    static bool log_inited = false;
    if (!log_inited) {
        nanolog::initialize(nanolog::GuaranteedLogger(), (home_dir / "log/").string(), "robot_server", 10, 10);
        nanolog::set_log_level(nanolog::LogLevel::INFO);
        log_inited = true;
    }

    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";
    return utils::GetRobotModelFromConfig(robot_cfg_path);
}

}  // namespace

TEST(JointImpedanceControllerTest, ZeroWrenchKeepsBaseReference) {
    RobotModel robot_model = LoadRobotModel();
    dynamics::JointImpedanceController controller(robot_model);

    VectorDof feedback_q = VectorDof::Zero();
    VectorDof feedback_dq = VectorDof::Zero();
    VectorDof cmd_q = VectorDof::Zero();
    VectorDof cmd_dq = VectorDof::Zero();
    VectorDof joint_stiffness = VectorDof::Constant(200.0);
    VectorDof joint_damping = VectorDof::Constant(20.0);

    auto output = controller.UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, cmd_dq, Vector12d::Zero(),
                                                 joint_stiffness, joint_damping, 0.01);

    EXPECT_EQ(output.force_trigger_state[0], 0);
    EXPECT_EQ(output.force_trigger_state[1], 0);
    EXPECT_NEAR((output.q_desired_compliant - cmd_q).norm(), 0.0, 1e-12);
    EXPECT_NEAR(output.joint_compliance_offset.norm(), 0.0, 1e-12);
    EXPECT_NEAR(output.torque_joint.norm(), 0.0, 1e-12);
}

TEST(JointImpedanceControllerTest, PrimaryWrenchProducesExternalTorque) {
    RobotModel robot_model = LoadRobotModel();
    dynamics::JointImpedanceController controller(robot_model);

    VectorDof feedback_q = VectorDof::Zero();
    VectorDof feedback_dq = VectorDof::Zero();
    VectorDof cmd_q = VectorDof::Zero();
    VectorDof cmd_dq = VectorDof::Zero();
    VectorDof joint_stiffness = VectorDof::Constant(200.0);
    VectorDof joint_damping = VectorDof::Constant(20.0);

    // Zero wrench baseline
    auto output_zero = controller.UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, cmd_dq, Vector12d::Zero(),
                                                      joint_stiffness, joint_damping, 0.01);
    controller.Reset();

    // Non-zero primary wrench
    Vector12d sensor_wrench = Vector12d::Zero();
    sensor_wrench[0] = 12.0;
    auto output = controller.UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, cmd_dq, sensor_wrench,
                                                 joint_stiffness, joint_damping, 0.01);

    // q_desired_compliant should equal cmd_q (no offset)
    EXPECT_NEAR((output.q_desired_compliant - cmd_q).norm(), 0.0, 1e-12);
    EXPECT_NEAR(output.joint_compliance_offset.norm(), 0.0, 1e-12);

    // torque_joint should differ from zero-wrench case due to -J^T * W_ext
    VectorDof torque_diff = output.torque_joint - output_zero.torque_joint;

    // Extract primary arm portion of torque difference — should be non-zero
    Eigen::VectorXd primary_diff;
    utils::GetJointFromWholeBody(robot_model, torque_diff, primary_diff, MotionGroup::Primary);
    EXPECT_GT(primary_diff.norm(), 0.0);

    // Secondary arm portion should be zero (secondary wrench is zero)
    Eigen::VectorXd secondary_diff;
    utils::GetJointFromWholeBody(robot_model, torque_diff, secondary_diff, MotionGroup::Secondary);
    EXPECT_NEAR(secondary_diff.norm(), 0.0, 1e-12);
}

TEST(JointImpedanceControllerTest, ComplianceDisabledIgnoresWrench) {
    RobotModel robot_model = LoadRobotModel();
    dynamics::JointImpedanceController controller_enabled(robot_model);
    dynamics::JointImpedanceController controller_disabled(robot_model);

    VectorDof feedback_q = VectorDof::Zero();
    VectorDof feedback_dq = VectorDof::Zero();
    VectorDof cmd_q = VectorDof::Zero();
    VectorDof cmd_dq = VectorDof::Zero();
    VectorDof joint_stiffness = VectorDof::Constant(200.0);
    VectorDof joint_damping = VectorDof::Constant(20.0);
    Vector12d sensor_wrench = Vector12d::Zero();
    sensor_wrench[0] = 12.0;

    // enable_compliance = false: wrench should have no effect
    auto output_disabled = controller_disabled.UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, cmd_dq,
                                                                   sensor_wrench, joint_stiffness, joint_damping,
                                                                   0.01, false);

    // enable_compliance = false with zero wrench should give same result
    dynamics::JointImpedanceController controller_zero(robot_model);
    auto output_zero = controller_zero.UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, cmd_dq,
                                                           Vector12d::Zero(), joint_stiffness, joint_damping,
                                                           0.01, false);

    EXPECT_NEAR((output_disabled.torque_joint - output_zero.torque_joint).norm(), 0.0, 1e-12);
}
