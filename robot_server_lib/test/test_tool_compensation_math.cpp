#include <gtest/gtest.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "pinocchio_api.h"
#include "tool_compensation.h"

using namespace robot_server_lib;

namespace {
std::string BuildArmUrdf(bool with_payload, bool rotated_base) {
    std::ostringstream urdf;
    urdf << R"(<robot name="test_arm"><link name="root"/>)";
    if (rotated_base) {
        urdf << R"(<link name="test_arm_base"/>
          <joint name="connector" type="fixed">
            <origin xyz="0.1 0.2 0.3" rpy="1.5707963267948966 0 0"/>
            <parent link="root"/><child link="test_arm_base"/>
          </joint>)";
    }
    const std::array<const char*, 7> axes{"0 0 1", "0 1 0", "1 0 0", "0 0 1", "0 1 0", "1 0 0", "0 1 0"};
    for (int joint = 1; joint <= 7; ++joint) {
        const std::string parent = joint == 1 ? (rotated_base ? "test_arm_base" : "root")
                                              : "link" + std::to_string(joint - 1);
        urdf << "<link name=\"link" << joint << R"("><inertial>
          <origin xyz="0.01 -0.02 0.03"/><mass value="1.0"/>
          <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.025"/>
          </inertial></link>)";
        urdf << "<joint name=\"j" << joint << "\" type=\"revolute\"><axis xyz=\"" << axes[joint - 1]
             << R"("/><origin xyz="0.09 0.02 0.08" rpy="0.1 0.2 -0.1"/>
          <limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/>)"
             << "<parent link=\"" << parent << "\"/><child link=\"link" << joint << "\"/></joint>";
    }
    urdf << R"(<link name="ee"><inertial><mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial></link>
      <joint name="ee_joint" type="fixed"><origin xyz="0.06 0.02 -0.01" rpy="0.2 -0.3 0.4"/>
        <parent link="link7"/><child link="ee"/></joint>)";
    if (with_payload) {
        urdf << R"(<link name="payload"><inertial>
          <origin xyz="0.015 -0.02 0.05"/><mass value="1.061"/>
          <inertia ixx="0.004" ixy="0.0001" ixz="0" iyy="0.006" iyz="0" izz="0.008"/>
          </inertial></link>
          <joint name="payload_joint" type="fixed"><parent link="ee"/><child link="payload"/></joint>)";
    }
    urdf << "</robot>";
    return urdf.str();
}

void ExpectRneaDifference(bool rotated_base) {
    pinocchio::Model arm, loaded_arm;
    pinocchio::urdf::buildModelFromXML(BuildArmUrdf(false, rotated_base), arm);
    pinocchio::urdf::buildModelFromXML(BuildArmUrdf(true, rotated_base), loaded_arm);
    ASSERT_EQ(arm.nq, 7);
    ASSERT_EQ(loaded_arm.nq, 7);
    arm.armature.setConstant(0.2);
    loaded_arm.armature = arm.armature;
    Matrix3d inertia = Vector3d(0.004, 0.006, 0.008).asDiagonal();
    inertia(0, 1) = inertia(1, 0) = 0.0001;
    dynamics::tool_compensation::ToolCompensation compensation(
        arm, pinocchio::Inertia(1.061, Vector3d(0.015, -0.02, 0.05), inertia));
    pinocchio::Data arm_data(arm), loaded_data(loaded_arm);
    const Eigen::VectorXd q = (Eigen::VectorXd(7) << 0.3, -0.5, 0.8, -0.4, 0.6, -0.3, 0.2).finished();
    const Eigen::VectorXd dq = (Eigen::VectorXd(7) << -0.2, 0.4, 0.6, -0.5, 0.3, -0.1, 0.7).finished();
    const Eigen::VectorXd ddq = (Eigen::VectorXd(7) << 0.5, -0.4, 0.2, 0.8, -0.3, 0.6, -0.7).finished();
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(7);
    const Matrix3d root_R_armbase = rotated_base
                                       ? Eigen::AngleAxisd(0.5 * std::acos(-1.0), Vector3d::UnitX()).toRotationMatrix()
                                       : Matrix3d::Identity();
    for (int sample = 0; sample < 4; ++sample) {
        const Vector3d gravity = sample < 2 ? Vector3d(1.1, -2.2, -9.1) : Vector3d::Zero();
        const Eigen::VectorXd q_sample = sample == 0 ? zero : q;
        const Eigen::VectorXd dq_sample = sample == 1 || sample == 3 ? dq : zero;
        const Eigen::VectorXd ddq_sample = sample == 1 || sample == 2 ? ddq : zero;
        arm.gravity.linear() = loaded_arm.gravity.linear() = root_R_armbase * gravity;
        const Eigen::VectorXd tau_arm = pinocchio::rnea(arm, arm_data, q_sample, dq_sample, ddq_sample);
        const Eigen::VectorXd tau_loaded =
            pinocchio::rnea(loaded_arm, loaded_data, q_sample, dq_sample, ddq_sample);
        const Eigen::VectorXd expected = tau_loaded - tau_arm;
        const Eigen::VectorXd actual = compensation.ComputeTorque(q_sample, dq_sample, ddq_sample, gravity);
        EXPECT_GT(expected.norm(), 1e-5) << "sample " << sample;
        EXPECT_LT((actual - expected).norm(), 1e-10) << "sample " << sample;
    }
}
}  // namespace

TEST(ToolCompensationMathTest, MatchesRneaDifferenceForGravityAccelerationAndVelocity) {
    ExpectRneaDifference(false);
}

TEST(ToolCompensationMathTest, RotatesArmBaseGravityThroughFixedConnector) {
    ExpectRneaDifference(true);
}

TEST(ToolCompensationMathTest, LoadsRootBodyAndMimicFingerInertiaAtNeutralPose) {
    const auto path = std::filesystem::temp_directory_path() /
                      ("test_tool_inertia_" + std::to_string(::getpid()) + ".urdf");
    {
        std::ofstream out(path);
        out << R"(<robot name="test_hand">
          <link name="root"><inertial><mass value="2"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
          </inertial></link>
          <link name="palm"><inertial><mass value="1"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
          </inertial></link>
          <joint name="mount" type="fixed"><origin xyz="1 0 0"/>
            <parent link="root"/><child link="palm"/></joint>
          <link name="driver"><inertial><origin xyz="1 0 0"/><mass value="0.5"/>
            <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
          </inertial></link>
          <joint name="driver_joint" type="revolute"><origin xyz="0 1 0"/><axis xyz="0 0 1"/>
            <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
            <parent link="palm"/><child link="driver"/></joint>
          <link name="finger"><inertial><origin xyz="1 0 0"/><mass value="0.5"/>
            <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
          </inertial></link>
          <joint name="finger_joint" type="revolute"><axis xyz="0 0 1"/>
            <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
            <mimic joint="driver_joint" multiplier="1" offset="1.5707963267948966"/>
            <parent link="driver"/><child link="finger"/></joint>
        </robot>)";
    }
    const pinocchio::Inertia inertia = dynamics::tool_compensation::LoadToolInertia(path);
    std::filesystem::remove(path);
    EXPECT_DOUBLE_EQ(inertia.mass(), 4.0);
    EXPECT_TRUE(inertia.lever().isApprox(Vector3d(0.625, 0.375, 0.0), 1e-12));
    Matrix3d expected;
    expected << 2.0575, -1.0625, 0.0, -1.0625, 2.0575, 0.0, 0.0, 0.0, 3.995;
    EXPECT_TRUE(inertia.inertia().matrix().isApprox(expected, 1e-12));
}
