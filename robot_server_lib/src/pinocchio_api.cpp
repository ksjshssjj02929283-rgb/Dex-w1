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

Eigen::MatrixXd GetJacobiMatrix(const Model& model, const Eigen::VectorXd& q)
{
    Eigen::MatrixXd J(6, model.nv);
    pinocchio::Data data(model);

    int ee_frame_id = model.getFrameId("ee_link");
    if (ee_frame_id == -1)
    {
        throw std::runtime_error("Frame 'ee_link' not found in model!");
    }

    computeFrameJacobian(model, data, q, ee_frame_id, pinocchio::LOCAL, J);
    return J;
}

}  // namespace pinocchio
