#include "pinocchio_api.h"

namespace pinocchio {

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
    // 这里约定调用方传入的是“末端 frame 位于最后”的模型，例如单臂 URDF。
    // 不要把这个规则直接用于 whole-body 模型；全身 URDF 的最后一帧可能是 head/chassis/connector。
    int frame_id = model.nframes - 1;
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
    pose = data.oMf[frame_id];
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
