#pragma once

#include "robot_server_lib/robot_server_base.h"

namespace robot_server_lib
{

class RobotServer : public RobotServerBase
{
   public:
    using RobotServerBase::RobotServerBase;

   protected:
    void InitExtraInterfaces() override;
    void ManagerThreadFunc() override;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_server_state_real_subscriber_;
    struct
    {
        VectorDof joint_position;
        bool is_updated = false;
    } real_robot_sync_;
};

}  // namespace robot_server_lib