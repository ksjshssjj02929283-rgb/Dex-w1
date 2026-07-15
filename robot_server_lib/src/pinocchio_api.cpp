#include "pinocchio_api.h"

namespace pinocchio {

namespace {
// 在模型帧列表中查找 arm_base 帧 (名称以 "_arm_base" 结尾)。
// 装配 URDF 在 root link 与第一个旋转关节之间插入带旋转的固定连接器关节,
// 导致 root 坐标系与 arm_base 坐标系不一致。找到该帧后可将 FK / Jacobian
// 结果从 root 系变换到 arm_base 系, 与旧版独立 arm URDF 保持一致。
// 若模型中不存在此类帧 (如旧版 URDF), 返回 -1, 函数行为不变。
int FindArmBaseFrameId(const Model& model) {
    const std::string suffix = "arm_base";
    for (FrameIndex i = 0; i < model.nframes; ++i) {
        const auto& name = model.frames[i].name;
        if (name.size() >= suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

// 将 6×nv 雅可比矩阵的参考坐标系从 root 旋转到 arm_base:
// J_armbase = blkdiag(R^T, R^T) * J_root, 其中 R = arm_base 在 root 系中的旋转。
// 仅对非 LOCAL 参考系有效: LOCAL 帧的雅可比在 EE 本体系下表达, 不受 base 旋转影响。
void RotateJacobianToArmBase(Eigen::MatrixXd& J, const Model& model, const Data& data) {
    int arm_base_id = FindArmBaseFrameId(model);
    if (arm_base_id < 0) return;
    const Eigen::Matrix3d Rt = data.oMf[arm_base_id].rotation().transpose();
    Eigen::MatrixXd tmp = Rt * J.topRows(3);
    J.topRows(3) = tmp;
    tmp = Rt * J.bottomRows(3);
    J.bottomRows(3) = tmp;
}
}  // namespace

void PrintModelInfo(const Model& model) {
    std::cout << "==== Model Info ====" << std::endl;
    std::cout << "Number of joints: " << model.nv << std::endl;
    for (int idx = 1; idx <= model.nv; ++idx) {
        std::cout << "Joint " << idx << ": " << model.names[idx] << std::endl;
    }
}

Eigen::Matrix4d GetFK(const Model& model, const Eigen::VectorXd& q) {
    SE3 pose;
    Data data(model);
    int frame_id = model.nframes - 1;
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);

    int arm_base_id = FindArmBaseFrameId(model);
    if (arm_base_id >= 0) {
        pose = data.oMf[arm_base_id].inverse() * data.oMf[frame_id];
    } else {
        pose = data.oMf[frame_id];
    }
    return pose.toHomogeneousMatrix();
}

Eigen::MatrixXd GetJacobiMatrix(const Model& model, const Eigen::VectorXd& q, ReferenceFrame frame) {
    int nv = model.nv;
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, nv);

    int ee_frame_id = model.nframes - 1;
    if (ee_frame_id < 0) {
        assert(false && "No frames found in model!");
        return J;
    }

    Data data(model);
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
    computeJointJacobians(model, data, q);

    computeFrameJacobian(model, data, q, ee_frame_id, frame, J);

    if (frame != LOCAL) {
        RotateJacobianToArmBase(J, model, data);
    }
    return J;
}

Eigen::MatrixXd GetJacobianMatrixDot(const Model& model,
                                     const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& dq,
                                     ReferenceFrame frame) {
    Data data(model);
    int ee_frame_id = model.nframes - 1;
    Eigen::MatrixXd J_dot(6, model.nv);
    if (ee_frame_id < 0) {
        assert(false && "No frames found in model!");
        return J_dot;
    }

    // 确保正向运动学、帧位姿与关节雅可比已更新
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
    computeJointJacobians(model, data, q);

    // 计算雅可比的时间导数
    computeJointJacobiansTimeVariation(model, data, q, dq);
    getFrameJacobianTimeVariation(model, data, ee_frame_id, frame, J_dot);

    if (frame != LOCAL) {
        RotateJacobianToArmBase(J_dot, model, data);
    }
    return J_dot;
}

SE3 GetFkByFrameId(const Model& model, const Eigen::VectorXd& q_urdf_in, int frame_id) {
    Data data(model);
    forwardKinematics(model, data, q_urdf_in);
    updateFramePlacements(model, data);

    return data.oMf[frame_id];
}

void PseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped) {
    const double eps = std::numeric_limits<double>::epsilon();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd sing_vals = svd.singularValues();

    int m = static_cast<int>(M_.rows());
    int n = static_cast<int>(M_.cols());
    int k = static_cast<int>(sing_vals.size());

    double max_s = (k > 0) ? sing_vals.maxCoeff() : 0.0;
    double tol = std::max(m, n) * max_s * eps;

    // adaptive damping parameter (can be tuned)
    double lambda = damped ? (1e-3 * max_s + 1e-9) : 0.0;

    // Construct S_pinv of size n x m
    Eigen::MatrixXd S_pinv = Eigen::MatrixXd::Zero(n, m);

    for (int i = 0; i < k; ++i) {
        double si = sing_vals(i);
        double inv_si = 0.0;
        if (damped) {
            // Tikhonov-regularized inverse of singular value:
            // inv = s / (s^2 + lambda^2)
            inv_si = si / (si * si + lambda * lambda);
        } else {
            // Moore-Penrose truncated inverse
            if (si > tol)
                inv_si = 1.0 / si;
            else
                inv_si = 0.0;
        }
        S_pinv(i, i) = inv_si;
    }

    // M_pinv = V * S_pinv * U^T
    M_pinv_ = svd.matrixV() * S_pinv * svd.matrixU().transpose();
}

Eigen::MatrixXd GetInertiaMatrix(const Model& model, const Eigen::VectorXd& q) {
    Data data(model);
    crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    return data.M;
}

}  // namespace pinocchio
