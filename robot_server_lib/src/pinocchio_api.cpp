#include "pinocchio_api.h"

namespace pinocchio
{

void PrintModelInfo(const Model& model)
{
    std::cout << "==== Model Info ====" << std::endl;
    std::cout << "Number of joints: " << model.nv << std::endl;
    for (int idx = 1; idx <= model.nv; ++idx)
    {
        std::cout << "Joint " << idx << ": " << model.names[idx] << std::endl;
    }
}

Eigen::Matrix4d GetFK(const Model& model, const Eigen::VectorXd &q)
{
    SE3 pose;
    Data data(model);
    int frame_id = model.getFrameId("ee_link");
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
    pose = data.oMf[frame_id];
    return pose.toHomogeneousMatrix();
}

Eigen::MatrixXd GetJacobiMatrix(const Model& model, const Eigen::VectorXd& q)
{
    Eigen::MatrixXd J(6, model.nv);

    int ee_frame_id = model.getFrameId("ee_link");
    if (ee_frame_id == -1)
    {
        assert(false && "Frame 'ee_link' not found in model!");
    }
    Data data(model);
    computeFrameJacobian(model, data, q, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
}

SE3 GetFkByFrameId(const Model& model, Data& data, const Eigen::VectorXd &q_urdf_in, int frame_id)
{
    forwardKinematics(model, data, q_urdf_in);
    updateFramePlacements(model, data);

    return data.oMf[frame_id];
}

void PseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped)
{
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

}  // namespace pinocchio
