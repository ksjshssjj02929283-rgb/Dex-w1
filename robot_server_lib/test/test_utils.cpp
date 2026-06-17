#include <gtest/gtest.h>

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
