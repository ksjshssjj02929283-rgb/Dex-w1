#include "torque_controller_base.h"

namespace robot_server_lib
{
TorqueControllerBase::TorqueControllerBase(RobotModel robot_model) : robot_model_(std::move(robot_model)) {}

TorqueControllerBase::~TorqueControllerBase() = default;

}  // namespace robot_server_lib
