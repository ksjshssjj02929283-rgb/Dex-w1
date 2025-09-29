#pragma once

#include "robot_server_lib/robot_server_base.h"

namespace robot_server_lib
{

class RobotServer : public RobotServerBase
{
   public:
    RobotServer(std::filesystem::path robot_cfg_path, bool is_sim);

   protected:
    void RobotThreadFunc() override;
    void ManagerThreadFunc() override;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_server_state_real_subscriber_;
    struct
    {
        VectorDof joint_position;
        bool is_updated = false;
    } real_robot_sync_;
};

}  // namespace robot_server_lib