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
}

TEST(JointImpedanceControllerTest, PrimaryWrenchTriggersPrimaryComplianceOnly) {
    RobotModel robot_model = LoadRobotModel();
    dynamics::JointImpedanceController controller(robot_model);

    VectorDof feedback_q = VectorDof::Zero();
    VectorDof feedback_dq = VectorDof::Zero();
    VectorDof cmd_q = VectorDof::Zero();
    VectorDof cmd_dq = VectorDof::Zero();
    VectorDof joint_stiffness = VectorDof::Constant(200.0);
    VectorDof joint_damping = VectorDof::Constant(20.0);
    Vector12d sensor_wrench = Vector12d::Zero();
    sensor_wrench[0] = 12.0;

    auto output = controller.UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, cmd_dq, sensor_wrench,
                                                 joint_stiffness, joint_damping, 0.01);

    Eigen::VectorXd primary_offset;
    Eigen::VectorXd secondary_offset;
    utils::GetJointFromWholeBody(robot_model, output.joint_compliance_offset, primary_offset, MotionGroup::Primary);
    utils::GetJointFromWholeBody(robot_model, output.joint_compliance_offset, secondary_offset, MotionGroup::Secondary);

    EXPECT_EQ(output.force_trigger_state[0], 1);
    EXPECT_EQ(output.force_trigger_state[1], 0);
    EXPECT_GT(primary_offset.norm(), 0.0);
    EXPECT_NEAR(secondary_offset.norm(), 0.0, 1e-12);
}
