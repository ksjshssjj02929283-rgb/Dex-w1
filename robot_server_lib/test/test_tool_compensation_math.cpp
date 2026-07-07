// 工具重力补偿数学验证:
// 对比 ComputeToolJointGravityTorque 的输出与 "把工具塞进 EE link 后 rnea 增量"
// 两者应完全一致 (差异来自 EE link 原有 50g 占位质量, 已扣除)
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "pinocchio_api.h"
#include "tool_compensation.h"
#include "typedef.h"

using namespace robot_server_lib;

namespace {
// 构造一个 7 自由度手臂 URDF, 末端 ee link 用占位质量
// 便于对比 "加工具前后 rnea 差值" 与 "ComputeToolJointGravityTorque"
std::string BuildArmUrdf(double ee_mass_kg, const std::string& ee_inertia = "") {
    std::string inertia_str =
        ee_inertia.empty() ? "ixx=\"1e-9\" ixy=\"0\" ixz=\"0\" iyy=\"1e-9\" iyz=\"0\" izz=\"1e-9\"" : ee_inertia;
    return R"(
<robot name="test_arm">
  <link name="base"/>
  <link name="link1"><inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="link2"><inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="link3"><inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="link4"><inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="link5"><inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="link6"><inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="link7"><inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="ee"><inertial><origin xyz="0 0 0.05" rpy="0 0 0"/><mass value=")" +
           std::to_string(ee_mass_kg) + R"("/><inertia )" + inertia_str + R"(/></inertial></link>
  <joint name="j1" type="revolute"><axis xyz="0 0 1"/><origin xyz="0 0 0.1"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/><parent link="base"/><child link="link1"/></joint>
  <joint name="j2" type="revolute"><axis xyz="0 1 0"/><origin xyz="0 0 0.1"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/><parent link="link1"/><child link="link2"/></joint>
  <joint name="j3" type="revolute"><axis xyz="0 1 0"/><origin xyz="0.1 0 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/><parent link="link2"/><child link="link3"/></joint>
  <joint name="j4" type="revolute"><axis xyz="0 0 1"/><origin xyz="0.1 0 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/><parent link="link3"/><child link="link4"/></joint>
  <joint name="j5" type="revolute"><axis xyz="0 1 0"/><origin xyz="0.1 0 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/><parent link="link4"/><child link="link5"/></joint>
  <joint name="j6" type="revolute"><axis xyz="0 0 1"/><origin xyz="0.1 0 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/><parent link="link5"/><child link="link6"/></joint>
  <joint name="j7" type="revolute"><axis xyz="0 1 0"/><origin xyz="0.05 0 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/><parent link="link6"/><child link="link7"/></joint>
  <joint name="ee_joint" type="fixed"><origin xyz="0.05 0 0" rpy="0 -1.5707963267948966 0"/><parent link="link7"/><child link="ee"/></joint>
</robot>
)";
}

// 写临时 URDF 文件
std::string WriteTempUrdf(double ee_mass_kg) {
    auto tmp_dir = std::filesystem::temp_directory_path();
    auto urdf_path = tmp_dir / ("test_arm_" + std::to_string(ee_mass_kg) + ".urdf");
    std::ofstream ofs(urdf_path);
    ofs << BuildArmUrdf(ee_mass_kg);
    ofs.close();
    return urdf_path.string();
}
}  // namespace

TEST(ToolCompensationMathTest, MatchesRneaDelta) {
    // 构造两个模型: base (ee=0kg) 和 with_tool (ee=1.061kg, com 在 ee 系 z=0.05m)
    pinocchio::Model model_base;
    pinocchio::Model model_with_tool;
    pinocchio::urdf::buildModel(WriteTempUrdf(0.0), model_base);
    pinocchio::urdf::buildModel(WriteTempUrdf(1.061), model_with_tool);
    ASSERT_EQ(model_base.nq, 7);
    ASSERT_EQ(model_with_tool.nq, 7);

    // 工具配置: com 在 EE 系下 (0,0,0.05)m, 与 URDF ee inertial origin 一致
    dynamics::tool_compensation::ToolGravityCompensationConfig cfg;
    cfg.gravity_enabled = true;
    cfg.mass_kg = 1.061;
    cfg.com_m = Vector3d(0.0, 0.0, 0.05);

    // arm base 系下重力 (本测试 arm-base = base link, 无旋转, 重力 = (0,0,-9.8))
    const Vector3d gravity_armbase(0.0, 0.0, -9.8);

    // 多个姿态测试
    std::vector<Eigen::VectorXd> test_qs;
    test_qs.push_back(Eigen::VectorXd::Zero(7));
    test_qs.push_back((Eigen::VectorXd(7) << 0.3, -0.5, 0.8, -0.4, 0.6, -0.3, 0.2).finished());
    test_qs.push_back((Eigen::VectorXd(7) << 1.0, 0.5, -1.0, 0.8, -0.5, 1.2, -0.7).finished());
    test_qs.push_back((Eigen::VectorXd(7) << -0.8, 1.2, 0.3, -1.5, 0.9, -0.4, 1.1).finished());

    const Eigen::VectorXd dq = Eigen::VectorXd::Zero(7);
    const Eigen::VectorXd ddq = Eigen::VectorXd::Zero(7);

    // rnea 的重力通过 model.gravity 设置
    model_base.gravity.linear(gravity_armbase);
    model_with_tool.gravity.linear(gravity_armbase);

    for (size_t i = 0; i < test_qs.size(); ++i) {
        const auto& q = test_qs[i];

        // 基准: 不带工具的 rnea (带重力)
        pinocchio::Data data_base(model_base);
        Eigen::VectorXd tau_base = pinocchio::rnea(model_base, data_base, q, dq, ddq);

        // 带工具: rnea (带重力)
        pinocchio::Data data_tool(model_with_tool);
        Eigen::VectorXd tau_with_tool = pinocchio::rnea(model_with_tool, data_tool, q, dq, ddq);

        // 工具引入的力矩增量 (rnea 差值)
        Eigen::VectorXd tau_delta = tau_with_tool - tau_base;

        // 我们的工具补偿函数
        Eigen::VectorXd tau_comp =
            dynamics::tool_compensation::ComputeToolJointGravityTorque(model_base, q, cfg, gravity_armbase);

        // 对比 (注意: pinocchio rnea 的重力模型, gravity 向量是"重力加速度", 与我们一致)
        double error = (tau_delta - tau_comp).cwiseAbs().maxCoeff();
        std::cout << "q[" << i << "]: tau_delta=" << tau_delta.transpose()
                  << "\n      tau_comp =" << tau_comp.transpose() << "\n      max_err=" << error << "\n";
        EXPECT_LT(error, 1e-9) << "q index " << i << ": tool comp mismatch";
    }
}
