#include "L1_server.h"

#include <cstdlib>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <system_error>

namespace robot_server_lib
{

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim) : RobotServerBase(robot_cfg_path, is_sim)
{
    // 创建 /hardware/joint_state 发布器
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/hardware/joint_state", 10);

    // 创建 /hardware/joint_command 订阅器 输入为28自由度, 16上半身+12腿部
    joint_command_subscriber_ = this->create_subscription<JointCommandMsg>(
        "/hardware/joint_command", 10, std::bind(&RobotServer::JointCommandCallback, this, std::placeholders::_1));

    // 创建 /control/debug 订阅器
    debug_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/control/debug", 10, std::bind(&RobotServer::DebugCallback, this, std::placeholders::_1));

    // 启动关节控制线程
    torque_control_thread_ = std::thread(&RobotServer::JointControlThreadFunc, this);
}

void RobotServer::JointCommandCallback(const JointCommandMsg::SharedPtr msg)
{
    // 将指令写入成员变量
    for (int i = 0; i < this->ndof_body_; ++i)
    {
        robot_control_.position[i] = msg->position[i];
        robot_control_.velocity[i] = msg->velocity[i];
        robot_control_.torque_feedforward[i] = msg->torque_feedforward[i];
    }
    for (int i = 0; i < this->ndof_legs_; ++i)
    {
        robot_control_.leg_control[i].joint_position = msg->position[i + ndof_body_];
        robot_control_.leg_control[i].joint_velocity = msg->velocity[i + ndof_body_];
        robot_control_.leg_control[i].joint_torque = msg->torque[i + ndof_body_];
        robot_control_.leg_control[i].joint_torque_feedforward = msg->torque_feedforward[i + ndof_body_];
        robot_control_.leg_control[i].joint_Kp = msg->stiffness[i + ndof_body_];
        robot_control_.leg_control[i].joint_Kd = msg->damping[i + ndof_body_];
    }
}

// 处理反馈数据和发送控制命令的实时线程函数
void RobotServer::JointControlThreadFunc()
{
    Setup_Realtime();
    // 初始化共享内存
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto& control_shm_instance = ControlSharedMemory::GetInstance();

    Int64 loop_counter = 0;
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        loop_counter++;
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);

        bool ret = feedback_shm_instance.PeekLatest(&robot_feedback_) && robot_feedback_.cnt > 0;
        // 根据状态写入机器人控制字
        this->is_robot_ready_ = false;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON;
        if (ret)
        {
            static bool inited = false;
            switch (robot_feedback_.robot_status_word)
            {
            case STATUSWORD_MOTOR_READY:
                // ready状态is_robot_ready_置为true, 可以写入measurement, 生成cmd后再使能电机
                this->is_robot_ready_ = true;
                this->WriteFeedbackToControl();
                this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
                inited = true;
                break;
            case STATUSWORD_MOTOR_ENABLED:
                this->is_robot_ready_ = true;
                this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
                if (!inited)
                {
                    this->WriteFeedbackToControl();
                    inited = true;
                }
                break;
            case STATUSWORD_MOTOR_NOT_READY:
            case STATUSWORD_MOTOR_FAULT:
            case STATUSWORD_MOTOR_QUICK_STOP:
            default:
                this->WriteFeedbackToControl();
                break;
            }

            if (!inited || this->clear_error_flag_)
            {
                this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CLEAR_FAULT;
                this->clear_error_flag_ = false;
            }
        }
        // 每10个周期publish一次关节状态
        if (loop_counter % 10 == 0)
        {
            this->JointStatePublisher();
            if (loop_counter % 500 == 0)
            {
                std::cout << "leg control command: \n"
                          << robot_control_.PrintLegControl() << std::endl;
            }

        }
        // 如果机器人未就绪则重置反馈共享内存
        if (!this->is_robot_ready_)
        {
            if (loop_counter % 5000 == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Robot is not ready, shm status: %d, statusword: %s", ret,
                            utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str());
                LOG_CRIT << "Robot is not ready, shm status: " << ret << ", statusword: " << robot_feedback_.robot_status_word;
            }
        }
        // 如果状态字发生变化了则打印
        static int16_t last_status_word_ = -1;
        static int16_t last_control_word_ = -1;
        if (robot_feedback_.robot_status_word != last_status_word_ || robot_control_.robot_control_word != last_control_word_)
        {
            RCLCPP_INFO(this->get_logger(), "Status word changed to: %s, control word changed to %s robot status: \n%s",
                        utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str(),
                        utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str().c_str(),
                        this->GetRobotStatusString().c_str());
            LOG_INFO << "";
            LOG_INFO << "Status word changed to " << utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str()
                     << ", control word changed to " << utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str()
                     << " robot status: " << this->GetRobotStatusString();
            last_status_word_ = robot_feedback_.robot_status_word;
            last_control_word_ = robot_control_.robot_control_word;
        }
        control_shm_instance.AppendNewElement(robot_control_);
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServer::WriteFeedbackToControl()
{
    for (int i = 0; i < this->ndof_body_; ++i)
    {
        robot_control_.position[i] = robot_feedback_.position[i];
        robot_control_.velocity[i] = 0.0;
        robot_control_.torque_feedforward[i] = 0.0;
    }
    for (int i = 0; i < this->ndof_legs_; ++i)
    {
        robot_control_.leg_control[i].joint_alias = robot_feedback_.leg_feedback_[i].joint_alias; // 关节ID初始化
        robot_control_.leg_control[i].joint_mode = 0; // 默认控制模式
        robot_control_.leg_control[i].joint_position = robot_feedback_.leg_feedback_[i].joint_position;
        robot_control_.leg_control[i].joint_velocity = 0.0;
        robot_control_.leg_control[i].joint_torque = 0.0;
        robot_control_.leg_control[i].joint_Kp = this->legs_stiffness_[i];  // 腿部刚度初始值
        robot_control_.leg_control[i].joint_Kd = this->legs_damping_[i];   // 腿部阻尼初始值
    }
}

void RobotServer::JointStatePublisher()
{
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.resize(this->ndof_body_ + this->ndof_legs_);
    joint_state_msg.position.resize(this->ndof_body_ + this->ndof_legs_);
    joint_state_msg.velocity.resize(this->ndof_body_ + this->ndof_legs_);
    joint_state_msg.effort.resize(this->ndof_body_ + this->ndof_legs_);
    for (int i = 0; i < this->ndof_body_; ++i)
    {
        joint_state_msg.position[i] = robot_feedback_.position[i];
        joint_state_msg.velocity[i] = robot_feedback_.velocity[i];
        joint_state_msg.effort[i] = robot_feedback_.torque[i];
    }
    for (int i = 0; i < this->ndof_legs_; ++i)
    {
        joint_state_msg.position[i + this->ndof_body_] = robot_feedback_.leg_feedback_[i].joint_position;
        joint_state_msg.velocity[i + this->ndof_body_] = robot_feedback_.leg_feedback_[i].joint_velocity;
        joint_state_msg.effort[i + this->ndof_body_] = robot_feedback_.leg_feedback_[i].joint_torque;
    }
    this->joint_state_publisher_->publish(joint_state_msg);
}

void RobotServer::DebugCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "/control/debug message received: %s", msg->data.c_str());
    LOG_INFO << "/control/debug message received: " << msg->data;

    try
    {
        auto parse_json = [](const nlohmann::json &obj, const char *key) -> bool
        {
            if (!obj.contains(key))
                return false;
            const auto &v = obj[key];
            if (v.is_boolean())
            {
                return v.get<bool>();
            }
            if (v.is_number_integer())
            {
                return v.get<int>();
            }
            return false;
        };
        if (parse_json(nlohmann::json::parse(msg->data), "clear_error"))
        {
            this->clear_error_flag_ = true;
            RCLCPP_INFO(this->get_logger(), "clear_error_flag_ set to true");
            LOG_INFO << "clear_error_flag_ set to true";
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to parse /control/debug JSON: %s", e.what());
    }
}

std::string RobotServer::GetRobotStatusString()
{
    uint64_t timestamp_ns = this->now().nanoseconds();
    auto status_string =
        std::string("{\n  \"timestamp\":") + std::to_string(timestamp_ns) +
        ",\n  \"robot_statusword\":" + utils::RobotStatuswordToJsonString(this->robot_feedback_.robot_status_word).str() +
        ", \"robot_controlword\":" + utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str() +
        ",\n  \"motor_status\":" + utils::MotorStatusToJsonArrayString(this->robot_feedback_.al_status, 0, ndof_body_ + ndof_legs_).str() +
        ",\n  \"motor_error_code\":" + utils::MotorErrorcodeToJsonArrayString(this->robot_feedback_.motor_error_code, 0, ndof_body_ + ndof_legs_).str() +
        ",\n  \"body_joint_position_cmd\":" + utils::VectorToJsonArrayString(robot_control_.position, 0, ndof_body_).str() +
        ",\n  \"body_joint_position\":" + utils::VectorToJsonArrayString(this->robot_feedback_.position, 0, ndof_body_).str() +
        ",\n  \"body_joint_velocity\":" + utils::VectorToJsonArrayString(this->robot_feedback_.velocity, 0, ndof_body_).str() +
        ",\n  \"body_joint_torque\":" + utils::VectorToJsonArrayString(this->robot_feedback_.torque, 0, ndof_body_).str() +
        ",\n  \"leg_control_cmd\":" + this->robot_control_.PrintLegControl() +
        "\n}";
    return status_string;
}

}  // namespace robot_server_lib
