#include "L1_server.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <system_error>

#include "parallel_ankle_calc.h"

namespace robot_server_lib {

RobotServer::RobotServer(std::filesystem::path robot_cfg_path, bool is_sim) : RobotServerBase(robot_cfg_path, is_sim) {
    // 尝试从 ~/dexe_robot_server_data/l1_config.json 加载配置
    std::filesystem::path config_dir = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    std::filesystem::path config_path = config_dir / "l1_config.json";

    if (std::filesystem::exists(config_path)) {
        try {
            std::ifstream config_file(config_path);
            nlohmann::json config = nlohmann::json::parse(config_file);
            config_file.close();

            // 读取 legs_stiffness
            if (config.contains("legs_stiffness") && config["legs_stiffness"].is_array()) {
                auto& arr = config["legs_stiffness"];
                int count = std::min(12, (int)arr.size());
                for (int i = 0; i < count; i++) {
                    legs_stiffness_[i] = arr[i].get<double>();
                }
                RCLCPP_INFO(this->get_logger(), "Loaded legs_stiffness from config file: %s",
                            config_path.string().c_str());
            } else {
                // 字段不存在，使用默认值并保存
                nlohmann::json arr_stiffness = nlohmann::json::array();
                for (int i = 0; i < 12; i++) arr_stiffness.push_back(legs_stiffness_[i]);
                config["legs_stiffness"] = arr_stiffness;
                std::ofstream out_file(config_path);
                out_file << config.dump(4);
                out_file.close();
                RCLCPP_INFO(this->get_logger(), "legs_stiffness not found in config, saved default values");
            }

            // 读取 legs_damping
            if (config.contains("legs_damping") && config["legs_damping"].is_array()) {
                auto& arr = config["legs_damping"];
                int count = std::min(12, (int)arr.size());
                for (int i = 0; i < count; i++) {
                    legs_damping_[i] = arr[i].get<double>();
                }
                RCLCPP_INFO(this->get_logger(), "Loaded legs_damping from config file: %s",
                            config_path.string().c_str());
            } else {
                // 字段不存在，使用默认值并保存
                nlohmann::json arr_damping = nlohmann::json::array();
                for (int i = 0; i < 12; i++) arr_damping.push_back(legs_damping_[i]);
                config["legs_damping"] = arr_damping;
                std::ofstream out_file(config_path);
                out_file << config.dump(4);
                out_file.close();
                RCLCPP_INFO(this->get_logger(), "legs_damping not found in config, saved default values");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load l1_config.json: %s, using default values", e.what());
            // 配置文件为空或解析失败，创建默认配置文件
            std::filesystem::create_directories(config_dir);
            nlohmann::json config;

            nlohmann::json arr_stiffness = nlohmann::json::array();
            for (int i = 0; i < 12; i++) arr_stiffness.push_back(legs_stiffness_[i]);
            config["legs_stiffness"] = arr_stiffness;

            nlohmann::json arr_damping = nlohmann::json::array();
            for (int i = 0; i < 12; i++) arr_damping.push_back(legs_damping_[i]);
            config["legs_damping"] = arr_damping;

            std::ofstream out_file(config_path);
            out_file << config.dump(4);
            out_file.close();
            RCLCPP_INFO(this->get_logger(), "Created default config file at: %s", config_path.string().c_str());
        }
    } else {
        // 配置文件不存在，创建默认配置文件
        std::filesystem::create_directories(config_dir);
        nlohmann::json config;

        nlohmann::json arr_stiffness = nlohmann::json::array();
        for (int i = 0; i < 12; i++) arr_stiffness.push_back(legs_stiffness_[i]);
        config["legs_stiffness"] = arr_stiffness;

        nlohmann::json arr_damping = nlohmann::json::array();
        for (int i = 0; i < 12; i++) arr_damping.push_back(legs_damping_[i]);
        config["legs_damping"] = arr_damping;

        std::ofstream out_file(config_path);
        out_file << config.dump(4);
        out_file.close();

        RCLCPP_INFO(this->get_logger(), "Config file not found, created default config at %s",
                    config_path.string().c_str());
    }

    // 打印当前的刚度和阻尼值
    std::string stiffness_str = "legs_stiffness: ";
    std::string damping_str = "legs_damping: ";
    for (int i = 0; i < 12; i++) {
        stiffness_str += std::to_string(legs_stiffness_[i]) + " ";
        damping_str += std::to_string(legs_damping_[i]) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", stiffness_str.c_str());
    RCLCPP_INFO(this->get_logger(), "%s", damping_str.c_str());

    // 创建 /hardware/joint_state 发布器
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/hardware/joint_state", 10);

    // 创建 /feedback/robot_server_state 发布器
    robot_status_publisher_ = this->create_publisher<std_msgs::msg::String>("/feedback/robot_server_state", 10);

    // 创建 /hardware/joint_command 订阅器 输入为28自由度, 16上半身+12腿部
    joint_command_subscriber_ = this->create_subscription<JointCommandMsg>(
        "/hardware/joint_command", 10, std::bind(&RobotServer::JointCommandCallback, this, std::placeholders::_1));

    joint_position_control_subscriber_ = this->create_subscription<joint_interfaces::msg::JointPositionControl>(
        "/hardware/joint_position_control", 10,
        std::bind(&RobotServer::JointPositionControlCallback, this, std::placeholders::_1));

    // 创建 /control/debug 订阅器
    debug_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/control/debug", 10, std::bind(&RobotServer::DebugCallback, this, std::placeholders::_1));

    // 启动关节控制线程
    torque_control_thread_ = std::thread(&RobotServer::JointControlThreadFunc, this);

    // 启动100Hz关节状态发布线程
    joint_state_publisher_thread_ = std::thread(&RobotServer::JointStatePublisherThreadFunc, this);
}

void RobotServer::JointPositionControlCallback(const joint_interfaces::msg::JointPositionControl::SharedPtr msg) {
    auto robot_control_temp = robot_control_;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_body_ + ndof_legs_; ++j) {
            if (joint_name_[j] == msg->name[i]) {
                if (j < ndof_body_) {
                    robot_control_temp.position[j] = msg->position[i];
                } else {
                    int leg_index = j - ndof_body_;
                    robot_control_temp.leg_control[leg_index].joint_position = msg->position[i];
                }
            }
        }
    }
    robot_control_ = robot_control_temp;
}

void RobotServer::JointCommandCallback(const JointCommandMsg::SharedPtr msg) {
    auto robot_control_temp = robot_control_;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_body_ + ndof_legs_; ++j) {
            if (joint_name_[j] == msg->name[i]) {
                int sign = 1;
                if (IsJointInMinusList(joint_name_[j])) {
                    sign = -1;
                }
                if (j < ndof_body_) {
                    robot_control_temp.position[j] = sign * msg->position[i];
                    robot_control_temp.velocity[j] = sign * msg->velocity[i];
                    robot_control_temp.torque_feedforward[j] = sign * msg->torque_feedforward[i];
                } else {
                    int leg_index = j - ndof_body_;
                    robot_control_temp.leg_control[leg_index].joint_position = sign * msg->position[i];
                    robot_control_temp.leg_control[leg_index].joint_velocity = sign * msg->velocity[i];
                    robot_control_temp.leg_control[leg_index].joint_torque = sign * msg->torque[i];
                    robot_control_temp.leg_control[leg_index].joint_torque_feedforward =
                        sign * msg->torque_feedforward[i];
                    robot_control_temp.leg_control[leg_index].joint_Kp = msg->stiffness[i];
                    robot_control_temp.leg_control[leg_index].joint_Kd = msg->damping[i];
                }
            }
        }
    }
    last_command_timestamp_ = msg->header.stamp;
    robot_control_ = robot_control_temp;
}

// 处理反馈数据和发送控制命令的实时线程函数
void RobotServer::JointControlThreadFunc() {
    Setup_Realtime();
    // 初始化共享内存
    auto& feedback_shm_instance = FeedbackSharedMemory::GetInstance();
    auto& control_shm_instance = ControlSharedMemory::GetInstance();

    Int64 loop_counter = 0;
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        rclcpp::Time current_time = this->get_clock()->now();
        rclcpp::Time last_time(last_command_timestamp_);
        rclcpp::Duration diff = current_time - last_time;
        loop_counter++;
        next_cycle_time_point += std::chrono::microseconds(torque_ctrl_dt_micro);
        // 根据状态写入机器人控制字
        this->is_robot_ready_ = false;
        robot_control_.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON;

        RobotFeedback robot_feedback_tmp;
        bool ret = feedback_shm_instance.PeekLatest(&robot_feedback_tmp) && robot_feedback_tmp.cnt > 0;
        if (ret) {
            // 换算脚踝关节
            robot_feedback_ = ConvertAnkleMotorsToPitchRoll(robot_feedback_tmp);

            static bool inited = false;
            switch (robot_feedback_.robot_status_word) {
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
                    if (!inited) {
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

            if (!inited || this->clear_error_flag_) {
                this->robot_control_.robot_control_word |= CTRLWORD_MOTOR_CLEAR_FAULT;
                this->clear_error_flag_ = false;
            }
        }
        // 如果机器人未就绪则重置反馈共享内存
        if (!this->is_robot_ready_) {
            if (loop_counter % 5000 == 0) {
                RCLCPP_WARN(this->get_logger(), "Robot is not ready, shm status: %d, statusword: %s", ret,
                            utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str());
                LOG_CRIT << "Robot is not ready, shm status: " << ret
                         << ", statusword: " << robot_feedback_.robot_status_word;
            }
        }
        // 如果指令超时则腿部指令置零
        if (last_command_timestamp_.sec == 0 && last_command_timestamp_.nanosec == 0 ||
            diff.nanoseconds() > static_cast<int64_t>(command_timeout_ms_) * 1000000LL) {
            for (int i = 0; i < 12; i++) {
                robot_control_.leg_control[i].joint_position = 0.0;
                robot_control_.leg_control[i].joint_velocity = 0.0;
                robot_control_.leg_control[i].joint_torque = 0.0;
                robot_control_.leg_control[i].joint_Kp = 0.0;
                robot_control_.leg_control[i].joint_Kd = 0.0;
            }
        }
        // 如果状态字发生变化了则打印
        static int16_t last_status_word_ = -1;
        static int16_t last_control_word_ = -1;
        if (robot_feedback_.robot_status_word != last_status_word_ ||
            robot_control_.robot_control_word != last_control_word_) {
            LOG_INFO << "";
            LOG_INFO << "Status word changed to "
                     << utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str()
                     << ", control word changed to "
                     << utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str()
                     << " robot status: " << InvertJointSignFromJsonString(this->GetRobotStatusString());
            RCLCPP_INFO(this->get_logger(), "Status word: %s, control word: %s",
                        utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str(),
                        utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str().c_str());
            last_status_word_ = robot_feedback_.robot_status_word;
            last_control_word_ = robot_control_.robot_control_word;
        }
        RobotControl robot_control_temp = ConvertPitchRollToAnkleMotors(robot_control_);
        control_shm_instance.AppendNewElement(robot_control_temp);
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

RobotFeedback RobotServer::ConvertAnkleMotorsToPitchRoll(const RobotFeedback& robot_feedback) {
    RobotFeedback result = robot_feedback;

    ankle_motor_torque_[0] = robot_feedback.leg_feedback_[4].joint_torque;
    ankle_motor_torque_[1] = robot_feedback.leg_feedback_[5].joint_torque;
    ankle_motor_torque_[2] = robot_feedback.leg_feedback_[10].joint_torque;
    ankle_motor_torque_[3] = robot_feedback.leg_feedback_[11].joint_torque;

    const double ankle_motor1_left = robot_feedback.leg_feedback_[4].joint_position;
    const double ankle_motor2_left = robot_feedback.leg_feedback_[5].joint_position;
    const double ankle_vel_motor1_left = robot_feedback.leg_feedback_[4].joint_velocity;
    const double ankle_vel_motor2_left = robot_feedback.leg_feedback_[5].joint_velocity;
    const double ankle_tau_motor1_left = robot_feedback.leg_feedback_[4].joint_torque;
    const double ankle_tau_motor2_left = robot_feedback.leg_feedback_[5].joint_torque;

    const double ankle_motor1_right = robot_feedback.leg_feedback_[10].joint_position;
    const double ankle_motor2_right = robot_feedback.leg_feedback_[11].joint_position;
    const double ankle_vel_motor1_right = robot_feedback.leg_feedback_[10].joint_velocity;
    const double ankle_vel_motor2_right = robot_feedback.leg_feedback_[11].joint_velocity;
    const double ankle_tau_motor1_right = robot_feedback.leg_feedback_[10].joint_torque;
    const double ankle_tau_motor2_right = robot_feedback.leg_feedback_[11].joint_torque;

    Eigen::Vector2d ankle_pose_left = pm_ankle_left_.GetAnklePoseFromMotorAngles(ankle_motor1_left, ankle_motor2_left);
    double ankle_pitch_left = ankle_pose_left(0);
    double ankle_roll_left = ankle_pose_left(1);

    Eigen::Vector2d ankle_pose_right =
        pm_ankle_right_.GetAnklePoseFromMotorAngles(ankle_motor1_right, ankle_motor2_right);
    double ankle_pitch_right = ankle_pose_right(0);
    double ankle_roll_right = ankle_pose_right(1);

    result.leg_feedback_[4].joint_position = ankle_pitch_left;
    result.leg_feedback_[5].joint_position = ankle_roll_left;
    result.leg_feedback_[10].joint_position = ankle_pitch_right;
    result.leg_feedback_[11].joint_position = ankle_roll_right;

    Eigen::Vector2d ankle_vel_left = pm_ankle_left_.GetTaskVelocitiesFromMotorVelocities(
        ankle_pitch_left, ankle_roll_left, ankle_vel_motor1_left, ankle_vel_motor2_left);
    Eigen::Vector2d ankle_vel_right = pm_ankle_right_.GetTaskVelocitiesFromMotorVelocities(
        ankle_pitch_right, ankle_roll_right, ankle_vel_motor1_right, ankle_vel_motor2_right);

    result.leg_feedback_[4].joint_velocity = ankle_vel_left(0);
    result.leg_feedback_[5].joint_velocity = ankle_vel_left(1);
    result.leg_feedback_[10].joint_velocity = ankle_vel_right(0);
    result.leg_feedback_[11].joint_velocity = ankle_vel_right(1);

    Eigen::Vector2d ankle_tau_left = pm_ankle_left_.GetTaskTorquesFromMotorTorques(
        ankle_pitch_left, ankle_roll_left, ankle_tau_motor1_left, ankle_tau_motor2_left);
    Eigen::Vector2d ankle_tau_right = pm_ankle_right_.GetTaskTorquesFromMotorTorques(
        ankle_pitch_right, ankle_roll_right, ankle_tau_motor1_right, ankle_tau_motor2_right);

    result.leg_feedback_[4].joint_torque = ankle_tau_left(0);
    result.leg_feedback_[5].joint_torque = ankle_tau_left(1);
    result.leg_feedback_[10].joint_torque = ankle_tau_right(0);
    result.leg_feedback_[11].joint_torque = ankle_tau_right(1);

    return result;
}

RobotControl RobotServer::ConvertPitchRollToAnkleMotors(const RobotControl& robot_control) {
    RobotControl result = robot_control;

    const double ankle_pitch_left = std::clamp(robot_control.leg_control[4].joint_position, -1.0472, 0.61087);
    const double ankle_roll_left = std::clamp(robot_control.leg_control[5].joint_position, -0.2618, 0.2618);
    const double ankle_tau_pitch_left = robot_control.leg_control[4].joint_torque;
    const double ankle_tau_roll_left = robot_control.leg_control[5].joint_torque;
    const double ankle_ff_pitch_left = robot_control.leg_control[4].joint_torque_feedforward;
    const double ankle_ff_roll_left = robot_control.leg_control[5].joint_torque_feedforward;
    const double ankle_k_pitch_left = robot_control.leg_control[4].joint_Kp;
    const double ankle_k_roll_left = robot_control.leg_control[5].joint_Kp;
    const double ankle_d_pitch_left = robot_control.leg_control[4].joint_Kd;
    const double ankle_d_roll_left = robot_control.leg_control[5].joint_Kd;

    const double ankle_pitch_right = std::clamp(robot_control.leg_control[10].joint_position, -1.0472, 0.61087);
    const double ankle_roll_right = std::clamp(robot_control.leg_control[11].joint_position, -0.2618, 0.2618);
    const double ankle_tau_pitch_right = robot_control.leg_control[10].joint_torque;
    const double ankle_tau_roll_right = robot_control.leg_control[11].joint_torque;
    const double ankle_ff_pitch_right = robot_control.leg_control[10].joint_torque_feedforward;
    const double ankle_ff_roll_right = robot_control.leg_control[11].joint_torque_feedforward;
    const double ankle_k_pitch_right = robot_control.leg_control[10].joint_Kp;
    const double ankle_k_roll_right = robot_control.leg_control[11].joint_Kp;
    const double ankle_d_pitch_right = robot_control.leg_control[10].joint_Kd;
    const double ankle_d_roll_right = robot_control.leg_control[11].joint_Kd;

    utils::AnkleIKSolution motor_angles_left = pm_ankle_left_.GetAnkleMotorAngles(ankle_pitch_left, ankle_roll_left);
    utils::AnkleIKSolution motor_angles_right =
        pm_ankle_right_.GetAnkleMotorAngles(ankle_pitch_right, ankle_roll_right);

    result.leg_control[4].joint_position = motor_angles_left.theta_1_rad;
    result.leg_control[5].joint_position = motor_angles_left.theta_2_rad;
    result.leg_control[10].joint_position = motor_angles_right.theta_1_rad;
    result.leg_control[11].joint_position = motor_angles_right.theta_2_rad;

    Eigen::Vector2d ankle_k_left = pm_ankle_left_.GetMotorStiffnessFromTaskStiffness(
        ankle_pitch_left, ankle_roll_left, ankle_k_pitch_left, ankle_k_roll_left);
    Eigen::Vector2d ankle_k_right = pm_ankle_right_.GetMotorStiffnessFromTaskStiffness(
        ankle_pitch_right, ankle_roll_right, ankle_k_pitch_right, ankle_k_roll_right);

    result.leg_control[4].joint_Kp = ankle_k_left(0);
    result.leg_control[5].joint_Kp = ankle_k_left(1);
    result.leg_control[10].joint_Kp = ankle_k_right(0);
    result.leg_control[11].joint_Kp = ankle_k_right(1);

    Eigen::Vector2d ankle_d_left = pm_ankle_left_.GetMotorDampingFromTaskDamping(ankle_pitch_left, ankle_roll_left,
                                                                                 ankle_d_pitch_left, ankle_d_roll_left);
    Eigen::Vector2d ankle_d_right = pm_ankle_right_.GetMotorDampingFromTaskDamping(
        ankle_pitch_right, ankle_roll_right, ankle_d_pitch_right, ankle_d_roll_right);

    result.leg_control[4].joint_Kd = ankle_d_left(0);
    result.leg_control[5].joint_Kd = ankle_d_left(1);
    result.leg_control[10].joint_Kd = ankle_d_right(0);
    result.leg_control[11].joint_Kd = ankle_d_right(1);

    Eigen::Vector2d ankle_tau_left = pm_ankle_left_.GetMotorTorquesFromTaskTorques(
        ankle_pitch_left, ankle_roll_left, ankle_tau_pitch_left, ankle_tau_roll_left);
    Eigen::Vector2d ankle_tau_right = pm_ankle_right_.GetMotorTorquesFromTaskTorques(
        ankle_pitch_right, ankle_roll_right, ankle_tau_pitch_right, ankle_tau_roll_right);

    result.leg_control[4].joint_torque = ankle_tau_left(0);
    result.leg_control[5].joint_torque = ankle_tau_left(1);
    result.leg_control[10].joint_torque = ankle_tau_right(0);
    result.leg_control[11].joint_torque = ankle_tau_right(1);

    Eigen::Vector2d ankle_ff_left = pm_ankle_left_.GetMotorTorquesFromTaskTorques(
        ankle_pitch_left, ankle_roll_left, ankle_ff_pitch_left, ankle_ff_roll_left);
    Eigen::Vector2d ankle_ff_right = pm_ankle_right_.GetMotorTorquesFromTaskTorques(
        ankle_pitch_right, ankle_roll_right, ankle_ff_pitch_right, ankle_ff_roll_right);

    result.leg_control[4].joint_torque_feedforward = ankle_ff_left(0);
    result.leg_control[5].joint_torque_feedforward = ankle_ff_left(1);
    result.leg_control[10].joint_torque_feedforward = ankle_ff_right(0);
    result.leg_control[11].joint_torque_feedforward = ankle_ff_right(1);

    return result;
}

void RobotServer::WriteFeedbackToControl() {
    for (int i = 0; i < this->ndof_body_; ++i) {
        robot_control_.position[i] = robot_feedback_.position[i];
        robot_control_.velocity[i] = 0.0;
        robot_control_.torque_feedforward[i] = 0.0;
        robot_control_.positive_torque_limit[i] = this->torque_limit_Nm_[i];
        robot_control_.negative_torque_limit[i] = -this->torque_limit_Nm_[i];
    }
    for (int i = 0; i < this->ndof_legs_; ++i) {
        robot_control_.leg_control[i].joint_alias = robot_feedback_.leg_feedback_[i].joint_alias;  // 关节ID初始化
        robot_control_.leg_control[i].joint_mode = 0;                                              // 默认控制模式
        robot_control_.leg_control[i].joint_position = robot_feedback_.leg_feedback_[i].joint_position;
        robot_control_.leg_control[i].joint_velocity = 0.0;
        robot_control_.leg_control[i].joint_torque = 0.0;
        robot_control_.leg_control[i].joint_Kp = this->legs_stiffness_[i];  // 腿部刚度初始值
        robot_control_.leg_control[i].joint_Kd = this->legs_damping_[i];    // 腿部阻尼初始值
    }
}

void RobotServer::JointStatePublisher() {
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.resize(this->ndof_body_ + this->ndof_legs_);
    joint_state_msg.position.resize(this->ndof_body_ + this->ndof_legs_);
    joint_state_msg.velocity.resize(this->ndof_body_ + this->ndof_legs_);
    joint_state_msg.effort.resize(this->ndof_body_ + this->ndof_legs_);
    int sign = 1;
    for (int i = 0; i < this->ndof_body_; ++i) {
        joint_state_msg.name[i] = this->joint_name_[i];
        IsJointInMinusList(joint_state_msg.name[i]) ? sign = -1 : sign = 1;
        joint_state_msg.position[i] = sign * robot_feedback_.position[i];
        joint_state_msg.velocity[i] = sign * robot_feedback_.velocity[i];
        joint_state_msg.effort[i] = sign * robot_feedback_.torque[i];
    }
    for (int i = 0; i < this->ndof_legs_; ++i) {
        joint_state_msg.name[i + this->ndof_body_] = this->joint_name_[i + this->ndof_body_];
        IsJointInMinusList(joint_name_[i + this->ndof_body_]) ? sign = -1 : sign = 1;
        joint_state_msg.position[i + this->ndof_body_] = sign * robot_feedback_.leg_feedback_[i].joint_position;
        joint_state_msg.velocity[i + this->ndof_body_] = sign * robot_feedback_.leg_feedback_[i].joint_velocity;
        joint_state_msg.effort[i + this->ndof_body_] = sign * robot_feedback_.leg_feedback_[i].joint_torque;
    }
    this->joint_state_publisher_->publish(joint_state_msg);

    // 发布 robot status string
    auto robot_status_msg = std_msgs::msg::String();
    robot_status_msg.data = this->GetRobotStatusString();
    this->robot_status_publisher_->publish(robot_status_msg);
}

void RobotServer::JointStatePublisherThreadFunc() {
    // 100Hz周期
    auto next_cycle_time_point = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        next_cycle_time_point += std::chrono::microseconds(publish_dt_micro);
        this->JointStatePublisher();
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

void RobotServer::DebugCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "/control/debug message received: %s", msg->data.c_str());
    LOG_INFO << "/control/debug message received: " << msg->data;

    try {
        auto parse_json = [](const nlohmann::json& obj, const char* key) -> bool {
            if (!obj.contains(key)) return false;
            const auto& v = obj[key];
            if (v.is_boolean()) {
                return v.get<bool>();
            }
            if (v.is_number_integer()) {
                return v.get<int>();
            }
            return false;
        };
        if (parse_json(nlohmann::json::parse(msg->data), "clear_error")) {
            this->clear_error_flag_ = true;
            RCLCPP_INFO(this->get_logger(), "clear_error_flag_ set to true");
            LOG_INFO << "clear_error_flag_ set to true";
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse /control/debug JSON: %s", e.what());
    }
}

std::string RobotServer::InvertJointSignFromJsonString(std::string status_string) {
    //将string变为json
    auto json_obj = nlohmann::json::parse(status_string);
    //获取joint_names
    auto joint_names = json_obj["joint_names"];
    //遍历joint name
    for (size_t i = 0; i < joint_names.size(); ++i) {
        std::string joint_name = joint_names[i].get<std::string>();
        if (IsJointInMinusList(joint_name)) {
            //判断是上半身还是下半身
            if (i < ndof_body_) {
                json_obj["body_joint_position"][i] = -json_obj["body_joint_position"][i].get<double>();
                json_obj["body_joint_velocity"][i] = -json_obj["body_joint_velocity"][i].get<double>();
                json_obj["body_joint_torque"][i] = -json_obj["body_joint_torque"][i].get<double>();
            } else {
                json_obj["leg_joint_position"][i - ndof_body_] =
                    -json_obj["leg_joint_position"][i - ndof_body_].get<double>();

                json_obj["leg_joint_velocity"][i - ndof_body_] =
                    -json_obj["leg_joint_velocity"][i - ndof_body_].get<double>();
                json_obj["leg_joint_torque"][i - ndof_body_] =
                    -json_obj["leg_joint_torque"][i - ndof_body_].get<double>();
            }
        }
    }
    //转换回string
    return json_obj.dump();
}

std::string RobotServer::GetRobotStatusString() {
    uint64_t timestamp_ns = this->now().nanoseconds();
    // 构建腿部位置数组
    std::vector<double> leg_position(ndof_legs_), leg_velocity(ndof_legs_), leg_torque(ndof_legs_),
        leg_stiffness(ndof_legs_), leg_damping(ndof_legs_);
    for (int i = 0; i < ndof_legs_; ++i) {
        leg_position[i] = robot_feedback_.leg_feedback_[i].joint_position;
        leg_velocity[i] = robot_feedback_.leg_feedback_[i].joint_velocity;
        leg_torque[i] = robot_feedback_.leg_feedback_[i].joint_torque;
        leg_stiffness[i] = robot_control_.leg_control[i].joint_Kp;
        leg_damping[i] = robot_control_.leg_control[i].joint_Kd;
    }
    // 构建脚踝电机原始力矩数组
    std::vector<double> leg_ankle_motor_torque = {
        ankle_motor_torque_[0], ankle_motor_torque_[1],
        ankle_motor_torque_[2], ankle_motor_torque_[3]
    };
    // 构建关节名称 JSON 数组
    std::string joint_names_str = "[";
    for (int i = 0; i < ndof_body_ + ndof_legs_; ++i) {
        if (i > 0) joint_names_str += ",";
        joint_names_str += "\"" + joint_name_[i] + "\"";
    }
    joint_names_str += "]";
    auto status_string =
        std::string("{\n  \"timestamp\":") + std::to_string(timestamp_ns) + ",\n  \"joint_names\":" + joint_names_str +
        ",\n  \"robot_statusword\":" +
        utils::RobotStatuswordToJsonString(this->robot_feedback_.robot_status_word).str() + ", \"robot_controlword\":" +
        utils::RobotControlwordToJsonString(this->robot_control_.robot_control_word).str() + ",\n  \"motor_status\":" +
        utils::MotorStatusToJsonArrayString(this->robot_feedback_.al_status, 0, ndof_body_ + ndof_legs_).str() +
        ",\n  \"motor_error_code\":" +
        utils::MotorErrorcodeToJsonArrayString(this->robot_feedback_.motor_error_code, 0, ndof_body_ + ndof_legs_)
            .str() +
        ",\n  \"body_joint_position_cmd\":" +
        utils::VectorToJsonArrayString(this->robot_control_.position, 0, ndof_body_).str() +
        ",\n  \"body_joint_position\":" +
        utils::VectorToJsonArrayString(this->robot_feedback_.position, 0, ndof_body_).str() +
        ",\n  \"body_joint_velocity\":" +
        utils::VectorToJsonArrayString(this->robot_feedback_.velocity, 0, ndof_body_).str() +
        ",\n  \"body_joint_torque\":" +
        utils::VectorToJsonArrayString(this->robot_feedback_.torque, 0, ndof_body_).str() +
        ",\n  \"leg_joint_position_cmd\":" +
        utils::VectorToJsonArrayString(this->robot_control_.position, ndof_body_, ndof_body_ + ndof_legs_).str() +
        ",\n  \"leg_joint_position\":" + utils::VectorToJsonArrayString(leg_position, 0, ndof_legs_).str() +
        ",\n  \"leg_joint_velocity\":" + utils::VectorToJsonArrayString(leg_velocity, 0, ndof_legs_).str() +
        ",\n  \"leg_joint_torque\":" + utils::VectorToJsonArrayString(leg_torque, 0, ndof_legs_).str() +
        ",\n  \"leg_stiffness\":" + utils::VectorToJsonArrayString(leg_stiffness, 0, ndof_legs_).str() +
        ",\n  \"leg_damping\":" + utils::VectorToJsonArrayString(leg_damping, 0, ndof_legs_).str() +
        ",\n  \"leg_ankle_motor_torque\":" + utils::VectorToJsonArrayString(leg_ankle_motor_torque, 0, 4).str() + "\n}";
    return InvertJointSignFromJsonString(status_string);
}

}  // namespace robot_server_lib
