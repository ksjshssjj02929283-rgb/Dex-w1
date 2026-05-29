#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>

#include "joint_impedance_controller.h"
#include "utils.h"

using namespace robot_server_lib;

namespace {

RobotModel LoadRobotModel() {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot_config.yaml";
    return utils::GetRobotModelFromConfig(robot_cfg_path);
}

VectorDof ComputeJointImpedanceTorque(const RobotModel& robot_model, const Vector12d& sensor_wrench) {
    dynamics::JointImpedanceController controller(robot_model);
    VectorDof feedback_q = VectorDof::Zero();
    VectorDof feedback_dq = VectorDof::Zero();
    VectorDof cmd_q = VectorDof::Zero();
    VectorDof cmd_dq = VectorDof::Zero();
    VectorDof stiffness = VectorDof::Zero();
    VectorDof damping = VectorDof::Zero();
    VectorDof tau = VectorDof::Zero();
    controller.UpdateTorqueCommand(feedback_q, feedback_dq, cmd_q, cmd_dq, stiffness, damping, 0.0, sensor_wrench,
                                   tau);
    return tau;
}

}  // namespace

TEST(JointImpedanceControllerTest, ExternalWrenchInsideDeadbandProducesNoTorque) {
    const RobotModel robot_model = LoadRobotModel();

    Vector12d sensor_wrench = Vector12d::Zero();
    sensor_wrench.head<6>() << 2.9, -2.9, 0.0, 0.09, -0.09, 0.0;

    const VectorDof tau = ComputeJointImpedanceTorque(robot_model, sensor_wrench);

    EXPECT_TRUE(tau.isZero(1e-12)) << tau.transpose();
}

TEST(JointImpedanceControllerTest, ExternalWrenchUsesLocalJacobianAndSaturates) {
    const RobotModel robot_model = LoadRobotModel();

    Vector12d sensor_wrench = Vector12d::Zero();
    sensor_wrench.head<6>() << 150.0, -120.0, 4.0, 15.0, -20.0, 0.2;

    const VectorDof tau = ComputeJointImpedanceTorque(robot_model, sensor_wrench);

    Eigen::VectorXd q_arm = Eigen::VectorXd::Zero(robot_model.primary_arm_model_ptr->nv);
    const Eigen::MatrixXd jacobian_local =
        pinocchio::GetJacobiMatrix(*robot_model.primary_arm_model_ptr, q_arm, pinocchio::LOCAL);
    Vector6d processed_wrench;
    processed_wrench << 100.0, -100.0, 1.0, 10.0, -10.0, 0.1;
    const Eigen::VectorXd tau_arm = jacobian_local.transpose() * processed_wrench;
    VectorDof expected_tau = VectorDof::Zero();
    utils::MapJointToWholeBody(robot_model, tau_arm, expected_tau, MotionGroup::Primary);

    EXPECT_TRUE(tau.isApprox(expected_tau, 1e-9)) << "tau: " << tau.transpose()
                                                 << "\nexpected: " << expected_tau.transpose();
}
