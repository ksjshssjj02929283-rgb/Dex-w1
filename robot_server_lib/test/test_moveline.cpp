#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_server_interfaces/action/move_robot.hpp>
#include <string>
#include <thread>
#include <vector>

class MoveLineTest : public rclcpp::Node
{
   public:
    using MoveRobot = robot_server_interfaces::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

    explicit MoveLineTest(bool is_sim) : rclcpp::Node("move_line_test")
    {
        const std::string action_name = is_sim ? "/control/move_robot_sim" : "/control/move_robot";
        action_client_ = rclcpp_action::create_client<MoveRobot>(this, action_name);
    }

    bool MoveLine(const std::string& motion_group, const std::vector<float>& target,
                  const std::string& config_json = R"({"reference_frame":"ArmBase","speed":1.0})")
    {
        using namespace std::chrono_literals;

        MoveRobot::Goal goal;
        goal.type = "MoveLine";
        goal.motion_group = motion_group;
        goal.target.assign(target.begin(), target.end());
        goal.config = config_json;

        rclcpp_action::Client<MoveRobot>::SendGoalOptions options;
        options.feedback_callback = [this](GoalHandleMoveRobot::SharedPtr, const std::shared_ptr<const MoveRobot::Feedback> feedback)
        { RCLCPP_INFO(this->get_logger(), "Feedback request_id: %ld", static_cast<long>(feedback->request_id)); };
        options.result_callback = [this](const GoalHandleMoveRobot::WrappedResult& result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Result: SUCCEEDED, request_id=%ld, success=%s", static_cast<long>(result.result->request_id),
                            result.result->success ? "true" : "false");
            }
            else if (result.code == rclcpp_action::ResultCode::ABORTED)
            {
                RCLCPP_ERROR(this->get_logger(), "Result: ABORTED");
            }
            else if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_ERROR(this->get_logger(), "Result: CANCELED");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Result: UNKNOWN");
            }
            rclcpp::shutdown();
        };

        auto goal_handle_future = action_client_->async_send_goal(goal, options);

        const auto wait_timeout = 5s;
        if (goal_handle_future.valid())
        {
            if (goal_handle_future.wait_for(wait_timeout) != std::future_status::ready)
            {
                RCLCPP_ERROR(get_logger(), "Timed out waiting for goal handle future");
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Invalid goal handle future");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal rejected");
            return false;
        }

        RCLCPP_INFO(get_logger(), "MoveLine goal sent: group=%s, target size=%zu", motion_group.c_str(), target.size());
        return true;
    }

   private:
    rclcpp_action::Client<MoveRobot>::SharedPtr action_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const bool is_sim = false;  // 设置为 true 以连接仿真机器人，false 以连接真实机器人
    auto node = std::make_shared<MoveLineTest>(is_sim);
    std::thread spin_thread([node]() { rclcpp::spin(node); });

    // 选择要发送的点位: 位置+姿态(四元数)：x y z qw qx qy qz
    std::vector<float> target_ee_pose_left_arm = {
        -0.13096, 0.38542,  0.39215,            // 位置 (米)
        0.80224,  -0.56692, -0.092544, 0.16263  // 姿态 (单位四元数qw, qx, qy, qz)
    };

    std::vector<float> target_ee_pose_right_arm = {0.13096, 0.38542, 0.29215, 0.16263, 0.092544, 0.56692, 0.80224};
    std::vector<float> target_torso = {0.0, 1.0, 0.0};  // 躯干位置 x z ry

    node->MoveLine("Torso", target_torso);  // motion_group: Primary or Secondary 对应左臂和右臂
    // node->MoveLine("Secondary", target_ee_pose_right_arm);

    return 0;
}