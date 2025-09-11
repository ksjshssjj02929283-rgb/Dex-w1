#include "pinocchio_api.h"

namespace pinocchio
{

Eigen::MatrixXd GetJacobiMatrix(const Model& model, Data& data, const Eigen::VectorXd& q)
{
    Eigen::MatrixXd J(6, model.nv);
    int ee_frame_id = model.getFrameId("ee_link");
    pinocchio::computeFrameJacobian(model, data, q, ee_frame_id, pinocchio::LOCAL, J);
    return J;
}

}  // namespace pinocchio
