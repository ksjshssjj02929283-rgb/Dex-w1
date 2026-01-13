#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_server_interfaces/action/move_robot.hpp>
#include <string>
#include <thread>
#include <vector>

class MoveRobotTest : public rclcpp::Node {
public:
    using MoveRobot = robot_server_interfaces::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

    explicit MoveRobotTest(bool is_sim) : rclcpp::Node("move_line_test") {
        const std::string action_name = is_sim ? "/control/move_robot_sim" : "/control/move_robot";
        action_client_ = rclcpp_action::create_client<MoveRobot>(this, action_name);
    }

    bool MoveRobotRequest(const std::string& type,
                          const std::string& motion_group,
                          const std::vector<float>& target,
                          const std::string& config_json = R"({"reference_frame":"ArmBase","speed":1.0})") {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return false;
        }
        MoveRobot::Goal goal;
        goal.type = type;
        goal.motion_group = motion_group;
        goal.target = target;
        goal.config = config_json;
        action_client_->async_send_goal(goal);
        return true;
    }

private:
    rclcpp_action::Client<MoveRobot>::SharedPtr action_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const bool is_sim = false;
    auto node = std::make_shared<MoveRobotTest>(is_sim);
    std::thread spin_thread([node]() { rclcpp::spin(node); });

    std::vector<float> target_torso = {0.0, 1.0, 0.0};
    node->MoveRobotRequest("MoveLine", "Torso", target_torso);

    std::vector<float> target_traj = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, -0.3, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, -0.4, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, -0.7, -0.7, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8, -0.8, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9, -0.9, 0.0, 0.0, 0.0, 0.0, 0.0,
    };

    node->MoveRobotRequest("MoveTraj", "WholeBody", target_traj, R"({"interval":0.5,"endpoint_index":[2,5,7,9]})");

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}