#pragma once

#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <atomic>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "planner_manager.h"
#include "rclcpp/rclcpp.hpp"
#include "torque_controller_manager.h"

namespace robot_server_lib {
using StreamingCommandMsg = robot_server_interfaces::msg::StreamingCommand;
using MoveRobot = robot_server_interfaces::action::MoveRobot;
using SetProperty = robot_server_interfaces::srv::SetProperty;

class RobotServerBase : public rclcpp::Node {
public:
    RobotServerBase(std::filesystem::path robot_cfg_path, bool is_sim);
    ~RobotServerBase();

protected:
    virtual void ManagerThreadFunc();
    virtual void TorqueControlThreadFunc();
    virtual void RobotThreadFunc(){};
    virtual void PublishThreadFunc();
    virtual void SetPropertyCallback(const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request>,
                                     std::shared_ptr<robot_server_interfaces::srv::SetProperty::Response>);

    void Run();
    Int64 MakeNewRequestID(const PlannerRequest& planner_request);
    void StreamingCommandCallback(const robot_server_interfaces::msg::StreamingCommand::SharedPtr msg);

    void MoveRobotCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>>);
    void ReportRequestStatus(Int64 request_id,
                             RequestType request_type,
                             RequestStatus request_status,
                             Status manager_state);
    void Setup_Realtime(int priority = 99);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_server_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_server_state_real_subscriber_;
    rclcpp::Subscription<robot_server_interfaces::msg::StreamingCommand>::SharedPtr streaming_command_subscriber_;
    rclcpp::Service<robot_server_interfaces::srv::SetProperty>::SharedPtr set_property_srv_;
    rclcpp_action::Server<robot_server_interfaces::action::MoveRobot>::SharedPtr move_robot_action_server_;

    std::thread manager_thread_;
    std::thread publish_thread_;
    std::thread torque_control_thread_;
    std::thread robot_thread_;

    bool is_sim_{false};
    bool is_robot_ready_{true};
    RobotModel robot_model_;
    PlannerManagerBuffers manager_buffers_;
    TorqueControlBuffer robot_buffers_;
    Status manager_state_{Status::Init};
    Int64 latest_received_id_{-1};
    BoundedQueue<std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>>>
        request_handle_queue_{32};
    std::filesystem::path server_data_dir_ = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    std::filesystem::path home_dir = std::filesystem::path(std::getenv("HOME"));
    // 记录最新的 streaming command 输入的关节位置
    VectorDof joint_position_input_;
    // 预分配的JSON序列化缓冲区，避免每次循环分配
    std::vector<double> json_position_buffer_;
    std::vector<double> json_velocity_buffer_;
    std::vector<double> json_effort_buffer_;
    std::vector<double> json_sensor_wrench_buffer_;
    std::vector<double> json_ee_pose_buffer_;
    struct {
        std::atomic_int64_t error_drop{-1};
        std::atomic_int64_t queued{-1};
        std::atomic_int64_t finished{-1};
    } latest_report_id_;

private:
    std::string GetRobotServerStateJson(uint64_t timestamp_ns,
                                        const std::optional<std::vector<double>>& position = std::nullopt,
                                        const std::optional<std::vector<double>>& velocity = std::nullopt,
                                        const std::optional<std::vector<double>>& effort = std::nullopt,
                                        const std::optional<std::vector<double>>& sensor_wrench = std::nullopt,
                                        const std::optional<std::vector<double>>& ee_pose = std::nullopt,
                                        const std::optional<TorqueCtrlCommand>& torque_cmd = std::nullopt,
                                        const std::optional<PlannerManagerInfo>& planner_info = std::nullopt);
};
}  // namespace robot_server_lib
