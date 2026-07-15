#include "l1_server.h"

#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <system_error>

#include "parallel_ankle_calc.h"

namespace robot_server_lib {

RobotServer::RobotServer(const std::string& config_path) : Node("robot_server") {
    // 加载配置
    auto config = LoadConfig(config_path);
    run_mode_ = config.run_mode;
    // 用配置文件中的踝关节几何参数构造并联踝关节求解器
    pm_ankle_left_ = utils::PMAnkle(true, config.left_ankle_params);
    pm_ankle_right_ = utils::PMAnkle(false, config.right_ankle_params);
    RCLCPP_INFO(this->get_logger(), "Config loaded from %s, run_mode=%d", config_path.c_str(), run_mode_);

    // 初始化关节索引缓存
    idx_l_ankle_pitch_ = GetJointIndexByName("ANKLE_PITCH_L_J5");
    idx_l_ankle_roll_ = GetJointIndexByName("ANKLE_ROLL_L_J6");
    idx_r_ankle_pitch_ = GetJointIndexByName("ANKLE_PITCH_R_J5");
    idx_r_ankle_roll_ = GetJointIndexByName("ANKLE_ROLL_R_J6");

    if (idx_l_ankle_pitch_ == -1 || idx_l_ankle_roll_ == -1 || idx_r_ankle_pitch_ == -1 || idx_r_ankle_roll_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find ankle joint indices by name!");
        throw std::runtime_error("Ankle joint indices not found");
    }

    LOG_INFO << "";
    LOG_INFO
        << "---------------------------------------L1 Robot Server Initializing---------------------------------------";

    // 创建 /hardware/joint_state 发布器
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/hardware/joint_state", 10);

    // 创建 /feedback/robot_server_state 发布器
    robot_status_publisher_ = this->create_publisher<std_msgs::msg::String>("/feedback/robot_server_state", 10);

    // 创建 /hardware/joint_command 订阅器
    joint_command_subscriber_ = this->create_subscription<JointCommandMsg>(
        "/hardware/joint_command", 10, std::bind(&RobotServer::JointCommandCallback, this, std::placeholders::_1));

    // 创建 /control/debug 订阅器
    debug_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/control/debug", 10, std::bind(&RobotServer::DebugCallback, this, std::placeholders::_1));

    // 启动关节控制线程
    torque_control_thread_ = std::thread(&RobotServer::JointControlThreadFunc, this);

    // 启动100Hz关节状态发布线程
    joint_state_publisher_thread_ = std::thread(&RobotServer::JointStatePublisherThreadFunc, this);
}

RobotServer::~RobotServer() {
    if (torque_control_thread_.joinable()) torque_control_thread_.join();
    if (joint_state_publisher_thread_.joinable()) joint_state_publisher_thread_.join();
}

void RobotServer::JointCommandCallback(const JointCommandMsg::SharedPtr msg) {
    auto robot_control_temp = robot_control_;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (int j = 0; j < ndof_total_; ++j) {
            if (joint_name_[j] == msg->name[i]) {
                robot_control_temp.joint_control[j].joint_position = msg->position[i];
                robot_control_temp.joint_control[j].joint_velocity = msg->velocity[i];
                robot_control_temp.joint_control[j].joint_torque_feedforward = msg->torque_feedforward[i];
                robot_control_temp.joint_control[j].joint_torque = msg->torque[i];
                robot_control_temp.joint_control[j].joint_Kp = msg->stiffness[i];
                robot_control_temp.joint_control[j].joint_Kd = msg->damping[i];
            }
        }
    }
    last_command_timestamp_ = this->get_clock()->now();
    robot_control_ = robot_control_temp;
}

// 处理反馈数据和发送控制命令的实时线程函数
void RobotServer::JointControlThreadFunc() {
    SetupRealtime();
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
        if (ret && !SortFeedbackByJointName(robot_feedback_tmp)) {
            ret = false;
        }
        if (ret) {
            // 电机空间 → 用户空间：对 minus 关节取反 position/velocity/torque
            for (int i = 0; i < ndof_total_; ++i) {
                int sign = GetJointSign(i);
                robot_feedback_tmp.joint_feedback[i].joint_position *= sign;
                robot_feedback_tmp.joint_feedback[i].joint_velocity *= sign;
                robot_feedback_tmp.joint_feedback[i].joint_torque *= sign;
            }
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
                    // 上肢力矩模式
                    if (this->run_mode_ == 1) {
                        robot_control_.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                    }
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
            for (int i = 0; i < ndof_legs_; i++) {
                int idx = i + ndof_body_;
                robot_control_.joint_control[idx].joint_position = robot_feedback_.joint_feedback[idx].joint_position;
                robot_control_.joint_control[idx].joint_velocity = 0.0;
                robot_control_.joint_control[idx].joint_torque = 0.0;
                robot_control_.joint_control[idx].joint_torque_feedforward = 0.0;
                robot_control_.joint_control[idx].joint_Kp = 0.0;
                robot_control_.joint_control[idx].joint_Kd = 0.0;
            }
        }
        // 如果指令超时则腿部指令置零
        if (last_command_timestamp_.sec == 0 && last_command_timestamp_.nanosec == 0 ||
            diff.nanoseconds() > static_cast<int64_t>(command_timeout_ms_) * 1000000LL) {
            LOG_WARN_THROTTLE(1000) << "Command timeout, setting leg joint commands to zero";
            for (int i = 0; i < ndof_legs_; i++) {
                int idx = i + ndof_body_;
                robot_control_.joint_control[idx].joint_position = robot_feedback_.joint_feedback[idx].joint_position;
                robot_control_.joint_control[idx].joint_velocity = 0.0;
                robot_control_.joint_control[idx].joint_torque = 0.0;
                robot_control_.joint_control[idx].joint_torque_feedforward = 0.0;
                robot_control_.joint_control[idx].joint_Kp = 0.0;
                robot_control_.joint_control[idx].joint_Kd = 0.0;
            }
        }
        // 如果状态字发生变化了则打印
        static int16_t last_status_word_ = -1;
        static int16_t last_control_word_ = -1;
        if (robot_feedback_.robot_status_word != last_status_word_ ||
            robot_control_.robot_control_word != last_control_word_ || loop_counter % 2000 == 0) {
            LOG_INFO << "";
            LOG_INFO << "Status word changed to "
                     << utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str()
                     << ", control word changed to "
                     << utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str();
            RCLCPP_INFO(this->get_logger(), "Status word: %s, control word: %s",
                        utils::RobotStatuswordToJsonString(robot_feedback_.robot_status_word).str().c_str(),
                        utils::RobotControlwordToJsonString(robot_control_.robot_control_word).str().c_str());
            last_status_word_ = robot_feedback_.robot_status_word;
            last_control_word_ = robot_control_.robot_control_word;
        }
        RobotControl robot_control_temp = ConvertPitchRollToAnkleMotors(robot_control_);
        // 上肢 PD 控制：底层电机不支持 PD，在 server 侧手动计算并叠加到力矩前馈
        for (int i = 0; i < ndof_body_; ++i) {
            double pos_error =
                robot_control_.joint_control[i].joint_position - robot_feedback_.joint_feedback[i].joint_position;
            double vel_error =
                robot_control_.joint_control[i].joint_velocity - robot_feedback_.joint_feedback[i].joint_velocity;
            robot_control_temp.joint_control[i].joint_torque_feedforward =
                robot_control_.joint_control[i].joint_torque_feedforward +
                robot_control_.joint_control[i].joint_Kp * pos_error +
                robot_control_.joint_control[i].joint_Kd * vel_error;
        }
        // 保存实际下发的力矩前馈（含上肢 PD 叠加和踝关节耦合项，用户空间）
        for (int i = 0; i < ndof_total_; ++i) {
            joint_torque_feedforward_[i] = robot_control_temp.joint_control[i].joint_torque_feedforward;
            // 用户空间 → 电机空间：对 minus 关节取反 position/velocity/torque/torque_feedforward
            int sign = GetJointSign(i);
            robot_control_temp.joint_control[i].joint_position *= sign;
            robot_control_temp.joint_control[i].joint_velocity *= sign;
            robot_control_temp.joint_control[i].joint_torque *= sign;
            robot_control_temp.joint_control[i].joint_torque_feedforward *= sign;
        }
        control_shm_instance.AppendNewElement(robot_control_temp);
        std::this_thread::sleep_until(next_cycle_time_point);
    }
}

bool RobotServer::SortFeedbackByJointName(RobotFeedback& feedback) {
    const int ndof_total = ndof_total_;

    // 检测反馈数据中的有效关节数量
    int valid_count = 0;
    for (int i = 0; i < ndof_total; ++i) {
        if (std::strlen(feedback.joint_feedback[i].joint_name) > 0) {
            valid_count++;
        }
    }
    if (valid_count != ndof_total) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "SortFeedbackByJointName: joint count mismatch. Expected %d, got %d", ndof_total,
                              valid_count);
        LOG_CRIT_THROTTLE(5000) << "SortFeedbackByJointName: joint count mismatch. Expected " << ndof_total << ", got "
                                << valid_count;
        return false;
    }

    // 按 joint_name_ 顺序查找并重排
    JointFeedback sorted_joints[MAX_JOINT_NUM]{};
    int matched_count = 0;
    for (int target_idx = 0; target_idx < ndof_total; ++target_idx) {
        for (int src_idx = 0; src_idx < ndof_total; ++src_idx) {
            if (std::string(feedback.joint_feedback[src_idx].joint_name) == joint_name_[target_idx]) {
                sorted_joints[target_idx] = feedback.joint_feedback[src_idx];
                matched_count++;
                break;
            }
        }
    }
    if (matched_count != ndof_total) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "SortFeedbackByJointName: matched %d/%d",
                              matched_count, ndof_total);
        return false;
    }

    for (int i = 0; i < ndof_total; ++i) {
        feedback.joint_feedback[i] = sorted_joints[i];
    }
    return true;
}

void RobotServer::WriteFeedbackToControl() {
    for (int i = 0; i < this->ndof_body_; ++i) {
        robot_control_.joint_control[i].joint_position = robot_feedback_.joint_feedback[i].joint_position;
        robot_control_.joint_control[i].joint_velocity = 0.0;
        robot_control_.joint_control[i].joint_torque_feedforward = 0.0;
        robot_control_.joint_control[i].joint_positive_torque_limit = this->torque_limit_Nm_[i];
        robot_control_.joint_control[i].joint_negative_torque_limit = -this->torque_limit_Nm_[i];
    }
    for (int i = 0; i < this->ndof_legs_; ++i) {
        int idx = i + this->ndof_body_;
        std::strncpy(robot_control_.joint_control[idx].joint_name, robot_feedback_.joint_feedback[idx].joint_name,
                     sizeof(robot_control_.joint_control[idx].joint_name) - 1);
        robot_control_.joint_control[idx].joint_mode = 0;
        robot_control_.joint_control[idx].joint_position = robot_feedback_.joint_feedback[idx].joint_position;
        robot_control_.joint_control[idx].joint_velocity = 0.0;
        robot_control_.joint_control[idx].joint_torque = 0.0;
        robot_control_.joint_control[idx].joint_torque_feedforward = 0.0;
        robot_control_.joint_control[idx].joint_Kp = 0.0;
        robot_control_.joint_control[idx].joint_Kd = 0.0;
    }
}

void RobotServer::JointStatePublisher() {
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.resize(ndof_total_);
    joint_state_msg.position.resize(ndof_total_);
    joint_state_msg.velocity.resize(ndof_total_);
    joint_state_msg.effort.resize(ndof_total_);
    for (int i = 0; i < this->ndof_body_; ++i) {
        joint_state_msg.name[i] = this->joint_name_[i];
        joint_state_msg.position[i] = robot_feedback_.joint_feedback[i].joint_position;
        joint_state_msg.velocity[i] = robot_feedback_.joint_feedback[i].joint_velocity;
        joint_state_msg.effort[i] = robot_feedback_.joint_feedback[i].joint_torque;
    }
    for (int i = 0; i < this->ndof_legs_; ++i) {
        int idx = i + this->ndof_body_;
        joint_state_msg.name[idx] = this->joint_name_[idx];
        joint_state_msg.position[idx] = robot_feedback_.joint_feedback[idx].joint_position;
        joint_state_msg.velocity[idx] = robot_feedback_.joint_feedback[idx].joint_velocity;
        joint_state_msg.effort[idx] = robot_feedback_.joint_feedback[idx].joint_torque;
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

std::string RobotServer::GetRobotStatusString() {
    uint64_t timestamp_ns = this->now().nanoseconds();
    const int ndof_total = ndof_total_;

    // 从统一 joint_feedback / joint_control 数组中提取所有关节字段
    std::vector<double> joint_position(ndof_total), joint_velocity(ndof_total), joint_torque(ndof_total);
    std::vector<double> joint_position_cmd(ndof_total), joint_stiffness(ndof_total), joint_damping(ndof_total);
    std::vector<uint16_t> motor_error_code(ndof_total);
    for (int i = 0; i < ndof_total; ++i) {
        joint_position[i] = robot_feedback_.joint_feedback[i].joint_position;
        joint_velocity[i] = robot_feedback_.joint_feedback[i].joint_velocity;
        joint_torque[i] = robot_feedback_.joint_feedback[i].joint_torque;
        joint_position_cmd[i] = robot_control_.joint_control[i].joint_position;
        joint_stiffness[i] = robot_control_.joint_control[i].joint_Kp;
        joint_damping[i] = robot_control_.joint_control[i].joint_Kd;
        motor_error_code[i] = robot_feedback_.joint_feedback[i].joint_error_code;
    }

    // 构建脚踝电机原始数据
    std::vector<double> ankle_motor_torque = {ankle_motor_torque_[L_ANKLE_MOTOR_1], ankle_motor_torque_[L_ANKLE_MOTOR_2],
                                              ankle_motor_torque_[R_ANKLE_MOTOR_1], ankle_motor_torque_[R_ANKLE_MOTOR_2]};
    std::vector<double> ankle_motor_position_fb = {
        ankle_motor_position_fb_[L_ANKLE_MOTOR_1], ankle_motor_position_fb_[L_ANKLE_MOTOR_2],
        ankle_motor_position_fb_[R_ANKLE_MOTOR_1], ankle_motor_position_fb_[R_ANKLE_MOTOR_2]};
    std::vector<double> ankle_motor_position_cmd = {
        ankle_motor_position_cmd_[L_ANKLE_MOTOR_1], ankle_motor_position_cmd_[L_ANKLE_MOTOR_2],
        ankle_motor_position_cmd_[R_ANKLE_MOTOR_1], ankle_motor_position_cmd_[R_ANKLE_MOTOR_2]};
    // 构建关节名称 JSON 数组
    std::string joint_names_str = "[";
    for (int i = 0; i < ndof_total; ++i) {
        if (i > 0) joint_names_str += ",";
        joint_names_str += std::string("\"") + joint_name_[i] + "\"";
    }
    joint_names_str += "]";
    auto status_string =
        std::string("{\n  \"timestamp\":") + std::to_string(timestamp_ns) + ",\n  \"joint_names\":" + joint_names_str +
        ",\n  \"robot_statusword\":" +
        utils::RobotStatuswordToJsonString(this->robot_feedback_.robot_status_word).str() + ", \"robot_controlword\":" +
        utils::RobotControlwordToJsonString(this->robot_control_.robot_control_word).str() +
        ",\n  \"run_mode\":" + std::to_string(run_mode_) + ",\n  \"motor_status\":" +
        utils::MotorStatusToJsonArrayString(this->robot_feedback_.al_status, 0, ndof_total).str() +
        ",\n  \"motor_error_code\":" + utils::MotorErrorcodeToJsonArrayString(motor_error_code, 0, ndof_total).str() +
        ",\n  \"joint_position_cmd\":" + utils::VectorToJsonArrayString(joint_position_cmd, 0, ndof_total).str() +
        ",\n  \"joint_position\":" + utils::VectorToJsonArrayString(joint_position, 0, ndof_total).str() +
        ",\n  \"joint_velocity\":" + utils::VectorToJsonArrayString(joint_velocity, 0, ndof_total).str() +
        ",\n  \"joint_torque\":" + utils::VectorToJsonArrayString(joint_torque, 0, ndof_total).str() +
        ",\n  \"joint_torque_feedforward\":" +
        utils::VectorToJsonArrayString(joint_torque_feedforward_, 0, ndof_total).str() +
        ",\n  \"joint_stiffness\":" + utils::VectorToJsonArrayString(joint_stiffness, 0, ndof_total).str() +
        ",\n  \"joint_damping\":" + utils::VectorToJsonArrayString(joint_damping, 0, ndof_total).str() +
        ",\n  \"leg_ankle_motor_torque\":" + utils::VectorToJsonArrayString(ankle_motor_torque, 0, 4).str() +
        ",\n  \"leg_ankle_motor_position_fb\":" + utils::VectorToJsonArrayString(ankle_motor_position_fb, 0, 4).str() +
        ",\n  \"leg_ankle_motor_position_cmd\":" +
        utils::VectorToJsonArrayString(ankle_motor_position_cmd, 0, 4).str() + "\n}";
    return status_string;
}

void RobotServer::DebugCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "/control/debug message received: %s", msg->data.c_str());
    LOG_INFO << "/control/debug message received: " << msg->data;

    try {
        auto json_data = nlohmann::json::parse(msg->data);

        auto parse_json_bool = [](const nlohmann::json& obj, const char* key) -> bool {
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
        if (parse_json_bool(json_data, "clear_error")) {
            this->clear_error_flag_ = true;
            RCLCPP_INFO(this->get_logger(), "clear_error_flag_ set to true");
            LOG_INFO << "clear_error_flag_ set to true";
        }
        if (json_data.contains("run_mode") && json_data["run_mode"].is_number_integer()) {
            this->run_mode_ = json_data["run_mode"].get<int>();
            RCLCPP_INFO(this->get_logger(), "run_mode_ set to %d", run_mode_);
            LOG_INFO << "run_mode_ set to " << run_mode_;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse /control/debug JSON: %s", e.what());
    }
}

void RobotServer::SetupRealtime() {
    // 锁内存，避免分页抖动
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "mlockall failed: " << std::strerror(errno) << std::endl;
    }
    // 设置SCHED_FIFO实时调度
    sched_param sp{};
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::cerr << "sched_setscheduler failed: " << std::strerror(errno) << std::endl;
        LOG_CRIT << "sched_setscheduler failed, please run setcap!";
        assert(false && "sched_setscheduler failed, please run setcap!");
    }
}

RobotFeedback RobotServer::ConvertAnkleMotorsToPitchRoll(const RobotFeedback& robot_feedback) {
    RobotFeedback result = robot_feedback;

    ankle_motor_torque_[L_ANKLE_MOTOR_1] = robot_feedback.joint_feedback[idx_l_ankle_pitch_].joint_torque;
    ankle_motor_torque_[L_ANKLE_MOTOR_2] = robot_feedback.joint_feedback[idx_l_ankle_roll_].joint_torque;
    ankle_motor_torque_[R_ANKLE_MOTOR_1] = robot_feedback.joint_feedback[idx_r_ankle_pitch_].joint_torque;
    ankle_motor_torque_[R_ANKLE_MOTOR_2] = robot_feedback.joint_feedback[idx_r_ankle_roll_].joint_torque;

    ankle_motor_position_fb_[L_ANKLE_MOTOR_1] = robot_feedback.joint_feedback[idx_l_ankle_pitch_].joint_position;
    ankle_motor_position_fb_[L_ANKLE_MOTOR_2] = robot_feedback.joint_feedback[idx_l_ankle_roll_].joint_position;
    ankle_motor_position_fb_[R_ANKLE_MOTOR_1] = robot_feedback.joint_feedback[idx_r_ankle_pitch_].joint_position;
    ankle_motor_position_fb_[R_ANKLE_MOTOR_2] = robot_feedback.joint_feedback[idx_r_ankle_roll_].joint_position;

    const double ankle_motor1_left = robot_feedback.joint_feedback[idx_l_ankle_pitch_].joint_position;
    const double ankle_motor2_left = robot_feedback.joint_feedback[idx_l_ankle_roll_].joint_position;
    const double ankle_vel_motor1_left = robot_feedback.joint_feedback[idx_l_ankle_pitch_].joint_velocity;
    const double ankle_vel_motor2_left = robot_feedback.joint_feedback[idx_l_ankle_roll_].joint_velocity;
    const double ankle_tau_motor1_left = robot_feedback.joint_feedback[idx_l_ankle_pitch_].joint_torque;
    const double ankle_tau_motor2_left = robot_feedback.joint_feedback[idx_l_ankle_roll_].joint_torque;

    const double ankle_motor1_right = robot_feedback.joint_feedback[idx_r_ankle_pitch_].joint_position;
    const double ankle_motor2_right = robot_feedback.joint_feedback[idx_r_ankle_roll_].joint_position;
    const double ankle_vel_motor1_right = robot_feedback.joint_feedback[idx_r_ankle_pitch_].joint_velocity;
    const double ankle_vel_motor2_right = robot_feedback.joint_feedback[idx_r_ankle_roll_].joint_velocity;
    const double ankle_tau_motor1_right = robot_feedback.joint_feedback[idx_r_ankle_pitch_].joint_torque;
    const double ankle_tau_motor2_right = robot_feedback.joint_feedback[idx_r_ankle_roll_].joint_torque;

    Eigen::Vector2d ankle_pose_left = pm_ankle_left_.GetAnklePoseFromMotorAngles(ankle_motor1_left, ankle_motor2_left);
    double ankle_pitch_left = ankle_pose_left(0);
    double ankle_roll_left = ankle_pose_left(1);

    Eigen::Vector2d ankle_pose_right =
        pm_ankle_right_.GetAnklePoseFromMotorAngles(ankle_motor1_right, ankle_motor2_right);
    double ankle_pitch_right = ankle_pose_right(0);
    double ankle_roll_right = ankle_pose_right(1);

    result.joint_feedback[idx_l_ankle_pitch_].joint_position = ankle_pitch_left;
    result.joint_feedback[idx_l_ankle_roll_].joint_position = ankle_roll_left;
    result.joint_feedback[idx_r_ankle_pitch_].joint_position = ankle_pitch_right;
    result.joint_feedback[idx_r_ankle_roll_].joint_position = ankle_roll_right;

    Eigen::Vector2d ankle_vel_left = pm_ankle_left_.GetTaskVelocitiesFromMotorVelocities(
        ankle_pitch_left, ankle_roll_left, ankle_vel_motor1_left, ankle_vel_motor2_left);
    Eigen::Vector2d ankle_vel_right = pm_ankle_right_.GetTaskVelocitiesFromMotorVelocities(
        ankle_pitch_right, ankle_roll_right, ankle_vel_motor1_right, ankle_vel_motor2_right);

    result.joint_feedback[idx_l_ankle_pitch_].joint_velocity = ankle_vel_left(0);
    result.joint_feedback[idx_l_ankle_roll_].joint_velocity = ankle_vel_left(1);
    result.joint_feedback[idx_r_ankle_pitch_].joint_velocity = ankle_vel_right(0);
    result.joint_feedback[idx_r_ankle_roll_].joint_velocity = ankle_vel_right(1);

    Eigen::Vector2d ankle_tau_left = pm_ankle_left_.GetTaskTorquesFromMotorTorques(
        ankle_pitch_left, ankle_roll_left, ankle_tau_motor1_left, ankle_tau_motor2_left);
    Eigen::Vector2d ankle_tau_right = pm_ankle_right_.GetTaskTorquesFromMotorTorques(
        ankle_pitch_right, ankle_roll_right, ankle_tau_motor1_right, ankle_tau_motor2_right);

    result.joint_feedback[idx_l_ankle_pitch_].joint_torque = ankle_tau_left(0);
    result.joint_feedback[idx_l_ankle_roll_].joint_torque = ankle_tau_left(1);
    result.joint_feedback[idx_r_ankle_pitch_].joint_torque = ankle_tau_right(0);
    result.joint_feedback[idx_r_ankle_roll_].joint_torque = ankle_tau_right(1);

    return result;
}

RobotControl RobotServer::ConvertPitchRollToAnkleMotors(const RobotControl& robot_control) {
    RobotControl result = robot_control;

    ankle_motor_position_cmd_[L_ANKLE_MOTOR_1] = robot_control.joint_control[idx_l_ankle_pitch_].joint_position;
    ankle_motor_position_cmd_[L_ANKLE_MOTOR_2] = robot_control.joint_control[idx_l_ankle_roll_].joint_position;
    ankle_motor_position_cmd_[R_ANKLE_MOTOR_1] = robot_control.joint_control[idx_r_ankle_pitch_].joint_position;
    ankle_motor_position_cmd_[R_ANKLE_MOTOR_2] = robot_control.joint_control[idx_r_ankle_roll_].joint_position;

    const double ankle_pitch_left = std::clamp(robot_control.joint_control[idx_l_ankle_pitch_].joint_position, -1.0472, 0.61087);
    const double ankle_roll_left = std::clamp(robot_control.joint_control[idx_l_ankle_roll_].joint_position, -0.2618, 0.2618);
    const double ankle_tau_pitch_left = robot_control.joint_control[idx_l_ankle_pitch_].joint_torque;
    const double ankle_tau_roll_left = robot_control.joint_control[idx_l_ankle_roll_].joint_torque;
    const double ankle_ff_pitch_left = robot_control.joint_control[idx_l_ankle_pitch_].joint_torque_feedforward;
    const double ankle_ff_roll_left = robot_control.joint_control[idx_l_ankle_roll_].joint_torque_feedforward;
    const double ankle_k_pitch_left = robot_control.joint_control[idx_l_ankle_pitch_].joint_Kp;
    const double ankle_k_roll_left = robot_control.joint_control[idx_l_ankle_roll_].joint_Kp;
    const double ankle_d_pitch_left = robot_control.joint_control[idx_l_ankle_pitch_].joint_Kd;
    const double ankle_d_roll_left = robot_control.joint_control[idx_l_ankle_roll_].joint_Kd;

    const double ankle_pitch_right =
        std::clamp(robot_control.joint_control[idx_r_ankle_pitch_].joint_position, -1.0472, 0.61087);
    const double ankle_roll_right = std::clamp(robot_control.joint_control[idx_r_ankle_roll_].joint_position, -0.2618, 0.2618);
    const double ankle_tau_pitch_right = robot_control.joint_control[idx_r_ankle_pitch_].joint_torque;
    const double ankle_tau_roll_right = robot_control.joint_control[idx_r_ankle_roll_].joint_torque;
    const double ankle_ff_pitch_right = robot_control.joint_control[idx_r_ankle_pitch_].joint_torque_feedforward;
    const double ankle_ff_roll_right = robot_control.joint_control[idx_r_ankle_roll_].joint_torque_feedforward;
    const double ankle_k_pitch_right = robot_control.joint_control[idx_r_ankle_pitch_].joint_Kp;
    const double ankle_k_roll_right = robot_control.joint_control[idx_r_ankle_roll_].joint_Kp;
    const double ankle_d_pitch_right = robot_control.joint_control[idx_r_ankle_pitch_].joint_Kd;
    const double ankle_d_roll_right = robot_control.joint_control[idx_r_ankle_roll_].joint_Kd;

    utils::AnkleIKSolution motor_angles_left = pm_ankle_left_.GetAnkleMotorAngles(ankle_pitch_left, ankle_roll_left);
    utils::AnkleIKSolution motor_angles_right =
        pm_ankle_right_.GetAnkleMotorAngles(ankle_pitch_right, ankle_roll_right);

    result.joint_control[idx_l_ankle_pitch_].joint_position = motor_angles_left.theta_1_rad;
    result.joint_control[idx_l_ankle_roll_].joint_position = motor_angles_left.theta_2_rad;
    result.joint_control[idx_r_ankle_pitch_].joint_position = motor_angles_right.theta_1_rad;
    result.joint_control[idx_r_ankle_roll_].joint_position = motor_angles_right.theta_2_rad;

    Eigen::Matrix2d ankle_k_left = pm_ankle_left_.GetMotorStiffnessMatrixFromTaskStiffness(
        ankle_pitch_left, ankle_roll_left, ankle_k_pitch_left, ankle_k_roll_left);
    Eigen::Matrix2d ankle_k_right = pm_ankle_right_.GetMotorStiffnessMatrixFromTaskStiffness(
        ankle_pitch_right, ankle_roll_right, ankle_k_pitch_right, ankle_k_roll_right);

    result.joint_control[idx_l_ankle_pitch_].joint_Kp = ankle_k_left(0, 0);
    result.joint_control[idx_l_ankle_roll_].joint_Kp = ankle_k_left(1, 1);
    result.joint_control[idx_r_ankle_pitch_].joint_Kp = ankle_k_right(0, 0);
    result.joint_control[idx_r_ankle_roll_].joint_Kp = ankle_k_right(1, 1);

    Eigen::Matrix2d ankle_d_left = pm_ankle_left_.GetMotorDampingMatrixFromTaskDamping(
        ankle_pitch_left, ankle_roll_left, ankle_d_pitch_left, ankle_d_roll_left);
    Eigen::Matrix2d ankle_d_right = pm_ankle_right_.GetMotorDampingMatrixFromTaskDamping(
        ankle_pitch_right, ankle_roll_right, ankle_d_pitch_right, ankle_d_roll_right);

    result.joint_control[idx_l_ankle_pitch_].joint_Kd = ankle_d_left(0, 0);
    result.joint_control[idx_l_ankle_roll_].joint_Kd = ankle_d_left(1, 1);
    result.joint_control[idx_r_ankle_pitch_].joint_Kd = ankle_d_right(0, 0);
    result.joint_control[idx_r_ankle_roll_].joint_Kd = ankle_d_right(1, 1);

    Eigen::Vector2d ankle_tau_left = pm_ankle_left_.GetMotorTorquesFromTaskTorques(
        ankle_pitch_left, ankle_roll_left, ankle_tau_pitch_left, ankle_tau_roll_left);
    Eigen::Vector2d ankle_tau_right = pm_ankle_right_.GetMotorTorquesFromTaskTorques(
        ankle_pitch_right, ankle_roll_right, ankle_tau_pitch_right, ankle_tau_roll_right);

    result.joint_control[idx_l_ankle_pitch_].joint_torque = ankle_tau_left(0);
    result.joint_control[idx_l_ankle_roll_].joint_torque = ankle_tau_left(1);
    result.joint_control[idx_r_ankle_pitch_].joint_torque = ankle_tau_right(0);
    result.joint_control[idx_r_ankle_roll_].joint_torque = ankle_tau_right(1);

    Eigen::Vector2d ankle_ff_left = pm_ankle_left_.GetMotorTorquesFromTaskTorques(
        ankle_pitch_left, ankle_roll_left, ankle_ff_pitch_left, ankle_ff_roll_left);
    Eigen::Vector2d ankle_ff_right = pm_ankle_right_.GetMotorTorquesFromTaskTorques(
        ankle_pitch_right, ankle_roll_right, ankle_ff_pitch_right, ankle_ff_roll_right);

    // 计算电机角度误差（使用原始的电机角度反馈值）
    const double motor1_left_error = motor_angles_left.theta_1_rad - ankle_motor_position_fb_[L_ANKLE_MOTOR_1];
    const double motor2_left_error = motor_angles_left.theta_2_rad - ankle_motor_position_fb_[L_ANKLE_MOTOR_2];
    const double motor1_right_error = motor_angles_right.theta_1_rad - ankle_motor_position_fb_[R_ANKLE_MOTOR_1];
    const double motor2_right_error = motor_angles_right.theta_2_rad - ankle_motor_position_fb_[R_ANKLE_MOTOR_2];

    // 将交叉耦合项加到前馈力矩中
    result.joint_control[idx_l_ankle_pitch_].joint_torque_feedforward =
        ankle_ff_left(0) + ankle_k_left(0, 1) * motor2_left_error;
    result.joint_control[idx_l_ankle_roll_].joint_torque_feedforward =
        ankle_ff_left(1) + ankle_k_left(1, 0) * motor1_left_error;
    result.joint_control[idx_r_ankle_pitch_].joint_torque_feedforward =
        ankle_ff_right(0) + ankle_k_right(0, 1) * motor2_right_error;
    result.joint_control[idx_r_ankle_roll_].joint_torque_feedforward =
        ankle_ff_right(1) + ankle_k_right(1, 0) * motor1_right_error;

    return result;
}

}  // namespace robot_server_lib
