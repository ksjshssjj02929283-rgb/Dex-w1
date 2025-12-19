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

SE3 GetFK(const Model& model, const Eigen::VectorXd &q)
{
    SE3 pose;
    Data data(model);
    int frame_id = model.getFrameId("ee_link");
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
    return data.oMf[frame_id];
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

}  // namespace pinocchio
