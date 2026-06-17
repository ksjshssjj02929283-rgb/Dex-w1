#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "utils.h"

using namespace robot_server_lib;

TEST(UtilsTest, IsFiniteVectorRejectsNanAndInfinity) {
    Vector6d wrench = Vector6d::Zero();
    EXPECT_TRUE(utils::IsFiniteVector(wrench));

    wrench[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(utils::IsFiniteVector(wrench));

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    EXPECT_TRUE(utils::IsFiniteVector(rotation));

    rotation(1, 2) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(utils::IsFiniteVector(rotation));
}

TEST(UtilsTest, GetEERotationWorldFromPoseVectorParsesArmQuaternion) {
    Eigen::VectorXd ee_pose_world(14);
    ee_pose_world << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5);

    const auto primary = utils::GetEERotationWorldFromPoseVector(ee_pose_world, 0);
    ASSERT_TRUE(primary.rotation.has_value());
    EXPECT_TRUE(primary.rotation->isApprox(Eigen::Matrix3d::Identity(), 1e-12));

    const auto secondary = utils::GetEERotationWorldFromPoseVector(ee_pose_world, 1);
    ASSERT_TRUE(secondary.rotation.has_value());
    EXPECT_TRUE(
        secondary.rotation->isApprox(Eigen::AngleAxisd(M_PI / 2.0, Vector3d::UnitZ()).toRotationMatrix(), 1e-12));
}

TEST(UtilsTest, GetEERotationWorldFromPoseVectorRejectsInvalidInput) {
    Eigen::VectorXd too_short = Eigen::VectorXd::Zero(6);
    const auto short_result = utils::GetEERotationWorldFromPoseVector(too_short, 0);
    EXPECT_FALSE(short_result.rotation.has_value());
    EXPECT_EQ(short_result.reason, utils::EERotationParseError::InsufficientSize);
    EXPECT_EQ(short_result.required_size, 7);

    Eigen::VectorXd non_finite(7);
    non_finite << 0.0, 0.0, 0.0, 1.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0;
    const auto non_finite_result = utils::GetEERotationWorldFromPoseVector(non_finite, 0);
    EXPECT_FALSE(non_finite_result.rotation.has_value());
    EXPECT_EQ(non_finite_result.reason, utils::EERotationParseError::NonFiniteQuaternion);

    Eigen::VectorXd zero_norm(7);
    zero_norm.setZero();
    const auto zero_norm_result = utils::GetEERotationWorldFromPoseVector(zero_norm, 0);
    EXPECT_FALSE(zero_norm_result.rotation.has_value());
    EXPECT_EQ(zero_norm_result.reason, utils::EERotationParseError::NearZeroQuaternion);
}
