#pragma once

#include "kinematics_base.h"

namespace robot_server_lib
{
namespace kinematics
{

class SRSKinematics : public KinematicsBase
{
   public:
    SRSKinematics(const SRSParameters &srs_params) : KinematicsBase()
    {
        // Initialize the kinematics with the provided SRS parameters
        this->params_ = srs_params;
        this->ndof_ = 7;
        this->ik_weight_ = Eigen::VectorXd(this->ndof_);
        this->ik_weight_ << 1, 2, 2, 1, 1, 1, 1;
    }

    ~SRSKinematics() override = default;

    PoseQuaternion GetFK(const Eigen::VectorXd &q_in, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override
    {
        pinocchio::SE3 pose_se3 = pinocchio::SE3::Identity();
        for (int i = 0; i < this->params_.dh_params.rows(); ++i)
        {
            double d = this->params_.dh_params(i, 0);
            double alpha = this->params_.dh_params(i, 1);
            double a = this->params_.dh_params(i, 2);
            double theta = this->params_.dh_params(i, 3);
            if (i < q_in.size())
            {
                theta += q_in[i] * this->params_.rotation_directions[i];
            }
            pinocchio::SE3 T = this->_DHCalc(d, alpha, a, theta);
            pose_se3 = pose_se3 * T;
        }
        pinocchio::SE3 T_b_ob_se3(this->params_.T_b_ob);
        pinocchio::SE3 T_oe_e_se3(this->params_.T_oe_e);
        pose_se3 = T_b_ob_se3 * pose_se3 * T_oe_e_se3 * tcp_frame;

        PoseQuaternion pose_out;
        pose_out.position = pose_se3.translation();
        pose_out.orientation = Eigen::Quaterniond(pose_se3.rotation());
        return pose_out;
    }

    bool GetIK(const PoseQuaternion &pose_in, Eigen::VectorXd &q_in_and_out, const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override
    {
        assert(q_in_and_out.size() == this->ndof_);
        const auto &q_in = q_in_and_out;

        // remove the TCP frame from the pose
        pinocchio::SE3 pose_se3;
        pinocchio::SE3 T_b_ob_se3(this->params_.T_b_ob);
        pinocchio::SE3 T_oe_e_se3(this->params_.T_oe_e);
        pose_se3.translation() = pose_in.position;
        pose_se3.rotation() = pose_in.orientation.toRotationMatrix();

        pose_se3 = T_b_ob_se3.inverse() * pose_se3 * T_oe_e_se3.inverse() * tcp_frame.inverse();

        // 计算当前臂角
        double current_arm_angle = _CalculateAnglePhi(q_in, pose_se3);

        // 基于当前臂角采样
        Eigen::VectorXd nsparams = Eigen::VectorXd::LinSpaced(this->num_samples_, current_arm_angle - small_range_, current_arm_angle + small_range_);
        std::vector<Eigen::VectorXd> ik_solutions;

        auto solve_and_choose_solution = [&](const Eigen::VectorXd &params)
        {
            for (Int64 index = 0; index < params.size(); ++index)
            {
                Eigen::Matrix<double, 4, 7> joints_list;
                if (_ComputeInverseKinematics(pose_se3, q_in, params[index], joints_list))
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        ik_solutions.emplace_back(joints_list.row(j));
                    }
                }
            }
            return _ChooseIKSolution(ik_solutions, q_in);
        };

        if (solve_and_choose_solution(nsparams))
        {
            q_in_and_out = ik_solutions[0];
            return true;
        }

        // 无解则扩大采样范围再次求解
        LOG_WARN << "No IK solution found in the initial range, expanding search range.";
        ik_solutions.clear();
        Eigen::VectorXd nsparams_wide_left =
            Eigen::VectorXd::LinSpaced(5 * this->num_samples_, current_arm_angle - 10 * small_range_, current_arm_angle - small_range_);
        Eigen::VectorXd nsparams_wide_right =
            Eigen::VectorXd::LinSpaced(5 * this->num_samples_, current_arm_angle + small_range_, current_arm_angle + 10 * small_range_);

        Eigen::VectorXd nsparams_wide(10 * this->num_samples_);
        nsparams_wide << nsparams_wide_left, nsparams_wide_right;

        if (solve_and_choose_solution(nsparams_wide))
        {
            q_in_and_out = ik_solutions[0];
            return true;
        }
        return false;
    }

   private:
    bool _ComputeInverseKinematics(const pinocchio::SE3 &target_pose_se3, const Eigen::VectorXd &joint_seed, double nsparam,
                                   Eigen::Matrix<double, 4, 7> &joints_list) const
    {
        int elbow_config = _Sign(joint_seed(3)) * this->params_.rotation_directions[3];

        Eigen::Vector3d P_target = target_pose_se3.translation();
        Eigen::Matrix3d R_target = target_pose_se3.rotation();
        Eigen::Vector3d P02(0, 0, this->params_.link_lengths(0));  // Base to shoulder
        Eigen::Vector3d P67(0, 0, this->params_.dh_params(6, 0));  // Hand to end-effector

        Eigen::Vector3d P06 = P_target - R_target * P67;
        Eigen::Vector3d P26 = P06 - P02;

        Eigen::VectorXd joints = Eigen::VectorXd::Zero(7);
        // Calculate joint angles
        bool res = this->_CalculateJointAngles(P26, elbow_config, joints);
        if (!res)
        {
            return false;
        }

        // Calculate transformations
        pinocchio::SE3 T34 = this->_DHCalc(this->params_.dh_params(3, 0), this->params_.dh_params(3, 1), this->params_.dh_params(3, 2), joints(3));
        Eigen::Matrix3d R34 = T34.rotation();

        // Calculate reference plane
        Eigen::Vector3d V_v_to_sew;
        Eigen::Matrix3d R03_o;
        Eigen::VectorXd joint_v;
        res = this->_ReferencePlane(target_pose_se3, elbow_config, V_v_to_sew, R03_o, joint_v);
        if (!res)
        {
            return false;
        }

        // Calculate shoulder joint rotation matrices
        Eigen::Vector3d usw = P26.normalized();
        Eigen::Matrix3d skew_usw = this->_SkewSymmetric(usw);

        double angle_phi = nsparam;

        // Calculate rotation matrix R03
        Eigen::Matrix3d A_s = skew_usw * R03_o;
        Eigen::Matrix3d B_s = -skew_usw * skew_usw * R03_o;
        Eigen::Matrix3d C_s = (usw * usw.transpose()) * R03_o;
        Eigen::Matrix3d R03 = A_s * std::sin(angle_phi) + B_s * std::cos(angle_phi) + C_s;

        // Calculate shoulder joint angles
        double angle1[2], angle2[2], angle3[2];
        angle1[0] = std::atan2(R03(1, 1), R03(0, 1));
        angle1[1] = std::atan2(-R03(1, 1), -R03(0, 1));

        angle2[0] = _SafeAcos(R03(2, 1));
        angle2[1] = -_SafeAcos(R03(2, 1));

        angle3[0] = std::atan2(-R03(2, 2), -R03(2, 0));
        angle3[1] = std::atan2(R03(2, 2), R03(2, 0));

        // Calculate wrist joint angles
        Eigen::Matrix3d A_w = R34.transpose() * A_s.transpose() * target_pose_se3.rotation();
        Eigen::Matrix3d B_w = R34.transpose() * B_s.transpose() * target_pose_se3.rotation();
        Eigen::Matrix3d C_w = R34.transpose() * C_s.transpose() * target_pose_se3.rotation();

        // Calculate wrist rotation matrix R47
        Eigen::Matrix3d R47 = A_w * std::sin(angle_phi) + B_w * std::cos(angle_phi) + C_w;

        // Calculate wrist joint angles
        double angle5[2], angle6[2], angle7[2];
        angle5[0] = std::atan2(R47(1, 2), R47(0, 2));
        angle5[1] = std::atan2(-R47(1, 2), -R47(0, 2));
        angle6[0] = _SafeAcos(R47(2, 2));
        angle6[1] = -_SafeAcos(R47(2, 2));
        angle7[0] = std::atan2(R47(2, 1), -R47(2, 0));
        angle7[1] = std::atan2(-R47(2, 1), R47(2, 0));

        // Generate multiple sets of joints
        for (int64_t i = 0; i < 2; ++i)
        {
            for (int64_t j = 0; j < 2; ++j)
            {
                joints_list(i * 2 + j, 0) = (angle1[i] - this->params_.dh_params(0, 3)) * this->params_.rotation_directions[0];
                joints_list(i * 2 + j, 1) = (angle2[i] - this->params_.dh_params(1, 3)) * this->params_.rotation_directions[1];
                joints_list(i * 2 + j, 2) = (angle3[i] - this->params_.dh_params(2, 3)) * this->params_.rotation_directions[2];
                joints_list(i * 2 + j, 3) = (joints(3) - this->params_.dh_params(3, 3)) * this->params_.rotation_directions[3];
                joints_list(i * 2 + j, 4) = (angle5[j] - this->params_.dh_params(4, 3)) * this->params_.rotation_directions[4];
                joints_list(i * 2 + j, 5) = (angle6[j] - this->params_.dh_params(5, 3)) * this->params_.rotation_directions[5];
                joints_list(i * 2 + j, 6) = (angle7[j] - this->params_.dh_params(6, 3)) * this->params_.rotation_directions[6];
            }
        }

        return true;
    }

    bool _ChooseIKSolution(std::vector<Eigen::VectorXd> &ik_solutions, const Eigen::VectorXd &joints_seed) const
    {
        int ik_number = ik_solutions.size();
        if (0 == ik_number)
        {
            return false;
        }
        // 如果超过关节限位或关节角位置变化过大, 则删除该解
        ik_solutions.erase(std::remove_if(ik_solutions.begin(), ik_solutions.end(),
                                          [this, &joints_seed](const Eigen::VectorXd &solution)
                                          {
                                              for (int i = 0; i < this->ndof_; ++i)
                                              {
                                                  if (solution(i) < this->params_.q_min(i) || solution(i) > this->params_.q_max(i) ||
                                                      std::abs(solution(i) - joints_seed(i)) > this->joint_threshold_)
                                                  {
                                                      return true;  // 需要删除
                                                  }
                                              }
                                              return false;  // 保留此解
                                          }),
                           ik_solutions.end());
        if (ik_solutions.empty())
            return false;
        // 根据权重对解进行排序
        std::sort(ik_solutions.begin(), ik_solutions.end(),
                  [this, &joints_seed](const Eigen::VectorXd &a, const Eigen::VectorXd &b)
                  {
                      double temp_distance_a = 0.0;
                      double temp_distance_b = 0.0;
                      for (int i = 0; i < this->ndof_; ++i)
                      {
                          temp_distance_a += std::abs((a(i) - joints_seed(i)) * this->ik_weight_(i));
                          temp_distance_b += std::abs((b(i) - joints_seed(i)) * this->ik_weight_(i));
                      }
                      return temp_distance_a < temp_distance_b;
                  });
        return true;
    }

    bool _CalculateJointAngles(const Eigen::Vector3d &P_s_to_w, int elbow_GC4, Eigen::VectorXd &joints) const
    {
        double d_bs = this->params_.link_lengths(0);
        double d_se = this->params_.link_lengths(1);
        double d_ew = this->params_.link_lengths(2);
        // Check reachability and calculate elbow joint angle
        double norm_P26 = P_s_to_w.norm();

        double elbow_cos_angle = (norm_P26 * norm_P26 - d_se * d_se - d_ew * d_ew) / (2 * d_se * d_ew);
        if (std::abs(elbow_cos_angle) > 1.0)
        {
            // assert(false && "Elbow angle calculation out of bounds");
            return false;
        }
        joints(3) = elbow_GC4 * std::acos(elbow_cos_angle);

        // Calculate joint 1
        if (std::abs(P_s_to_w.z()) > 1e-6)
        {
            joints(0) = std::atan2(P_s_to_w.y(), P_s_to_w.x());
        }
        else
        {
            joints(0) = 0;
        }

        // Calculate joint 2
        double euclidean_norm = std::hypot(P_s_to_w.x(), P_s_to_w.y());
        double angle_phi = std::acos((d_se * d_se + norm_P26 * norm_P26 - d_ew * d_ew) / (2 * d_se * norm_P26));
        joints(1) = std::atan2(euclidean_norm, P_s_to_w.z()) + elbow_GC4 * angle_phi;

        return true;
    }

    bool _ReferencePlane(const pinocchio::SE3 &pose, int elbow_GC4, Eigen::Vector3d &V_v_to_sew, Eigen::Matrix3d &R03_v,
                         Eigen::VectorXd &joint_v) const
    {
        // Calculate the reference plane vector, rotation matrix from base to elbow,
        // and joint values
        Eigen::Vector3d P_target = pose.translation();
        Eigen::Vector3d P02(0, 0,
                            this->params_.link_lengths(0));        // Base to shoulder
        Eigen::Vector3d P67(0, 0, this->params_.dh_params(6, 0));  // Hand to end-effector

        Eigen::Vector3d P06 = P_target - pose.rotation() * P67;
        Eigen::Vector3d P26 = P06 - P02;

        // Calculate joint angles
        joint_v = Eigen::VectorXd::Zero(7);
        bool res = this->_CalculateJointAngles(P26, elbow_GC4, joint_v);
        if (!res)
        {
            return false;
        }

        // Lower arm transformation
        pinocchio::SE3 T34_v = this->_DHCalc(this->params_.dh_params(3, 0), this->params_.dh_params(3, 1), this->params_.dh_params(3, 2), joint_v(3));
        Eigen::Vector3d P34_v = T34_v.translation();

        // Calculate reference elbow position and normal vector to the reference
        // plane
        Eigen::Vector3d v1 = (P34_v - P02).normalized();
        Eigen::Vector3d v2 = (P06 - P02).normalized();

        V_v_to_sew = v1.cross(v2);  // The normal vector to the plane

        R03_v = Eigen::Matrix3d::Identity();
        for (int i = 0; i < 3; ++i)
        {
            R03_v *=
                this->_DHCalc(this->params_.dh_params(i, 0), this->params_.dh_params(i, 1), this->params_.dh_params(i, 2), joint_v(i)).rotation();
        }

        return true;
    }

    double _CalculateAnglePhi(const Eigen::VectorXd &joint_angles, const pinocchio::SE3 &current_pose)
    {
        // 计算肩到腕向量
        Eigen::Vector3d P_target = current_pose.translation();
        Eigen::Vector3d P02(0, 0, this->params_.link_lengths(0));  // Base to shoulder
        Eigen::Vector3d P67(0, 0, this->params_.dh_params(6, 0));  // Hand to end-effector

        Eigen::Vector3d P06 = P_target - current_pose.rotation() * P67;
        Eigen::Vector3d P26 = P06 - P02;

        Eigen::Vector3d usw = P26.normalized();

        // 计算参考平面旋转矩阵
        Eigen::Matrix3d R03_v;
        Eigen::VectorXd joint_v;
        int elbow_config = _Sign(joint_angles(3));
        Eigen::Vector3d V_v_to_sew;
        _ReferencePlane(current_pose, elbow_config, V_v_to_sew, R03_v, joint_v);

        // 计算实际R03
        Eigen::Matrix3d R03_actual = Eigen::Matrix3d::Identity();
        for (int i = 0; i < 3; ++i)
        {
            double theta = joint_angles(i) * this->params_.rotation_directions[i] + this->params_.dh_params(i, 3);
            R03_actual = R03_actual *
                         this->_DHCalc(this->params_.dh_params(i, 0), this->params_.dh_params(i, 1), this->params_.dh_params(i, 2), theta).rotation();
        }

        // 通过旋转矩阵与参考平面关系反解臂角
        Eigen::Matrix3d skew_usw = _SkewSymmetric(usw);
        Eigen::Matrix3d A_s = skew_usw * R03_v;
        Eigen::Matrix3d B_s = -skew_usw * skew_usw * R03_v;
        Eigen::Matrix3d C_s = (usw * usw.transpose()) * R03_v;

        // 解psi: R03_actual = A_s*sin(psi) + B_s*cos(psi) + C_s
        // 这里用trace近似
        double psi = std::atan2((R03_actual - C_s).cwiseProduct(A_s).sum(), (R03_actual - C_s).cwiseProduct(B_s).sum());
        return psi;
    }

    pinocchio::SE3 _DHCalc(double d, double alpha, double a, double theta) const
    {
        Eigen::Matrix3d Rot;
        Rot << std::cos(theta), -std::sin(theta) * std::cos(alpha), std::sin(theta) * std::sin(alpha), std::sin(theta),
            std::cos(theta) * std::cos(alpha), -std::cos(theta) * std::sin(alpha), 0, std::sin(alpha), std::cos(alpha);
        Eigen::Vector3d Trans(a * std::cos(theta), a * std::sin(theta), d);
        return pinocchio::SE3(Rot, Trans);
    }

    Eigen::Matrix3d _SkewSymmetric(const Eigen::Vector3d &vector) const
    {
        Eigen::Matrix3d skew_matrix;
        skew_matrix << 0, -vector.z(), vector.y(), vector.z(), 0, -vector.x(), -vector.y(), vector.x(), 0;
        return skew_matrix;
    }

    double _SafeAcos(double x) const
    {
        if (x < -1.0)
            x = -1.0;
        else if (x > 1.0)
            x = 1.0;
        return acos(x);
    }

    int _Sign(double value) const
    {
        if (value >= 0)
        {
            return 1;
        }
        else
        {
            return -1;
        }
    }

    int ndof_;
    SRSParameters params_;
    int num_samples_ = 50;
    double joint_threshold_ = M_PI / 180.0 * 30;
    double small_range_ = M_PI / 18000.0 * num_samples_;  // 每0.01度一个采样点
    Eigen::VectorXd ik_weight_;
};

}  // namespace kinematics
}  // namespace robot_server_lib
