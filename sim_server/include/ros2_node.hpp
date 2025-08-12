#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_server_interfaces/action/move_robot.hpp>
#include <robot_server_interfaces/msg/streaming_command.hpp>
#include <robot_server_interfaces/srv/set_property.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "robot_server_lib/planner_manager.h"
#include "robot_server_lib/torque_controller_manager.h"
#include "robot_server_lib/utilities.h"

class Ros2Node : public rclcpp::Node
{
   public:
    Ros2Node();
    ~Ros2Node();

   private:
    void InitSimulation();
    void InitRobotServer();
    void OnStreamingCommand(const robot_server_interfaces::msg::StreamingCommand::SharedPtr msg);
    void SetPropertyOnRequest(const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request>,
                              std::shared_ptr<robot_server_interfaces::srv::SetProperty::Response>);
    void HandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>>);
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const robot_server_interfaces::action::MoveRobot::Goal>);
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>>);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_server_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_server_state_real_subscriber_;
    rclcpp::Subscription<robot_server_interfaces::msg::StreamingCommand>::SharedPtr streaming_command_subscriber_;
    rclcpp::Service<robot_server_interfaces::srv::SetProperty>::SharedPtr set_property_srv_;
    rclcpp_action::Server<robot_server_interfaces::action::MoveRobot>::SharedPtr move_robot_action_server_;

    robot_server_lib::Int64 MakeNewRequestID(robot_server_lib::RequestType type);
    robot_server_lib::PlannerRequest ParseSetPropertyRequest(const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request);
    robot_server_lib::PlannerRequest ParseMoveRequest(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> action_handle);
    robot_server_lib::StreamingCommand ParseStreamingCommand(const std::shared_ptr<robot_server_interfaces::msg::StreamingCommand> msg);

    std::thread manager_thread_;
    std::thread simulation_thread_;
    std::thread publish_thread_;
    std::thread torque_control_thread_;
    
    robot_server_lib::RobotModel robot_model_;
    robot_server_lib::PlannerManagerBuffers manager_buffers_;
    robot_server_lib::TorqueControlBuffer robot_buffers_;
    robot_server_lib::Status manager_state_{robot_server_lib::Status::Init};
    robot_server_lib::Int64 latest_received_id_{-1};
    robot_server_lib::BoundedQueue<std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>>>
        request_handle_queue_{32};

    struct
    {
        std::atomic_int64_t error_drop{-1};
        std::atomic_int64_t queued{-1};
        std::atomic_int64_t finished{-1};
    } latest_report_id_;

    struct
    {
        robot_server_lib::VectorDof joint_position;
        bool is_updated = false;
    } real_robot_sync_;
};