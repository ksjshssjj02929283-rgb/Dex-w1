#include "utils.h"

namespace robot_server_lib {
namespace utils {

RobotModel GetRobotModelFromConfig(std::filesystem::path robot_cfg_path) {
    json config_json;
    RobotParams params;
    RobotModel robot_model;
    if (!std::filesystem::exists(robot_cfg_path)) {
        LOG_CRIT << "Robot configuration file not found: " << robot_cfg_path;
        assert(false && "Robot configuration file not found");
    }
    std::ifstream ifs(robot_cfg_path);
    if (!ifs.is_open() || !(ifs >> config_json)) {
        LOG_CRIT << "Can't open or parse robot.cfg: " << robot_cfg_path;
        assert(false && "Can't open or parse robot.cfg");
    }
    const auto package_share_path = ament_index_cpp::get_package_share_directory("robot_server_lib");
    if (!config_json.contains("robot_name") || !config_json["robot_name"].is_string()) {
        LOG_CRIT << "robot.cfg missing 'robot_name' field";
        std::cout << "[Error] robot.cfg missing 'robot_name' field\n";
        assert(false && "robot.cfg missing 'robot_name' field");
    }
    params.robot_name = config_json["robot_name"].get<std::string>();

    if (config_json.contains("robot_version") && config_json["robot_version"].is_string()) {
        params.robot_version = config_json["robot_version"].get<std::string>();
    } else {
        LOG_CRIT << "robot.cfg missing 'robot_version' field, using default empty string";
        std::cout << "[Error] robot.cfg missing 'robot_version' field\n";
        assert(false && "robot.cfg missing 'robot_version' field");
    }

    params.urdf_path =
        std::filesystem::path(package_share_path) / "robot_models" / params.robot_name / (params.robot_name + ".urdf");

    if (config_json.contains("left_arm_name")) {
        params.left_arm_name = config_json["left_arm_name"].get<std::string>();
        params.left_arm_urdf_path = std::filesystem::path(package_share_path) / "robot_models" / params.left_arm_name /
                                    (params.left_arm_name + ".urdf");
    }
    if (config_json.contains("right_arm_name")) {
        params.right_arm_name = config_json["right_arm_name"].get<std::string>();
        params.right_arm_urdf_path = std::filesystem::path(package_share_path) / "robot_models" /
                                     params.right_arm_name / (params.right_arm_name + ".urdf");
    }

    auto parse_array = [&](const std::string &key, Eigen::VectorXd &dst) {
        if (config_json.contains(key)) {
            auto arr = config_json[key];
            dst = Eigen::VectorXd::Zero(arr.size());
            for (size_t i = 0; i < arr.size(); ++i) {
                dst[i] = arr[i].get<double>();
            }
        }
    };

    parse_array("joints_primary", params.joints_primary);
    parse_array("joints_secondary", params.joints_secondary);
    parse_array("joints_torso", params.joints_torso);
    parse_array("joint_max_velocity_limit_positive", params.joint_max_velocity_limit_positive);
    parse_array("joint_max_velocity_limit_negative", params.joint_max_velocity_limit_negative);
    parse_array("joint_max_torque_limit_positive", params.joint_max_torque_limit_positive);
    parse_array("joint_max_torque_limit_negative", params.joint_max_torque_limit_negative);

    // 解度析动力学基参数矩阵
    if (config_json.contains("dyn_base_params_primary")) {
        Eigen::VectorXd dyn_base_params_primary;
        parse_array("dyn_base_params_primary", dyn_base_params_primary);
        params.dyn_base_params_primary = dyn_base_params_primary;
    } else {
        std::cout << "Use dafault dynamic model in urdf\n";
        LOG_INFO << "Use dafault dynamic model in urdf";
    }
    if (config_json.contains("dyn_base_params_secondary")) {
        Eigen::VectorXd dyn_base_params_secondary;
        parse_array("dyn_base_params_secondary", dyn_base_params_secondary);
        params.dyn_base_params_secondary = dyn_base_params_secondary;
    } else {
        std::cout << "Use dafault dynamic model in urdf\n";
        LOG_INFO << "Use dafault dynamic model in urdf";
    }

    if (config_json.contains("joint_command_sequence_by_name")) {
        auto arr = config_json["joint_command_sequence_by_name"];
        params.joint_command_sequence_by_name.clear();
        for (const auto &item : arr) {
            params.joint_command_sequence_by_name.push_back(item.get<std::string>());
        }
    }

    pinocchio::Model model, left_arm_model, right_arm_model;
    pinocchio::urdf::buildModel(params.urdf_path, model);
    pinocchio::urdf::buildModel(params.left_arm_urdf_path, left_arm_model);
    pinocchio::urdf::buildModel(params.right_arm_urdf_path, right_arm_model);

    robot_model.model_ptr = std::make_shared<pinocchio::Model>(model);
    robot_model.primary_arm_model_ptr = std::make_shared<pinocchio::Model>(left_arm_model);
    robot_model.secondary_arm_model_ptr = std::make_shared<pinocchio::Model>(right_arm_model);
    robot_model.params_ptr = std::make_shared<RobotParams>(params);

    return robot_model;
}

void SetMax(std::atomic_int64_t &maximum_value, std::int64_t value) {
    std::int64_t prev_value = maximum_value;
    while (prev_value < value && !maximum_value.compare_exchange_weak(prev_value, value)) {
    }
}

PoseQuaternion VectorXdToPoseQuaternion(const Eigen::VectorXd &vec) {
    PoseQuaternion pose;
    pose.position = Eigen::Vector3d(vec[0], vec[1], vec[2]);
    pose.orientation = Eigen::Quaterniond(vec[3], vec[4], vec[5], vec[6]);
    pose.orientation.normalize();
    return pose;
}

PoseQuaternion StringToPoseQuaternion(const std::string &str) {
    PoseQuaternion pose;
    std::stringstream ss(str);
    std::string item;
    double values[7] = {0};
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 7) {
        values[idx++] = std::stod(item);
    }
    pose.position = Eigen::Vector3d(values[0], values[1], values[2]);
    pose.orientation = Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
    pose.orientation.normalize();
    return pose;
}

Vector3d StringToVector3d(const std::string &str) {
    std::stringstream ss(str);
    std::string item;
    Vector3d values;
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 3) {
        values[idx++] = std::stod(item);
    }
    return values;
}

Vector6d StringToVector6d(const std::string &str) {
    std::stringstream ss(str);
    std::string item;
    Vector6d values;
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 6) {
        values[idx++] = std::stod(item);
    }
    return values;
}

Eigen::VectorXd StringToVectorXd(const std::string &str) {
    // 根据逗号分隔字符串并转换为Eigen::VectorXd
    std::stringstream ss(str);
    std::string item;
    std::vector<double> values;
    while (std::getline(ss, item, ',')) {
        values.push_back(std::stod(item));
    }
    Eigen::VectorXd vec(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
        vec[i] = values[i];
    }
    return vec;
}

bool MapJointToWholeBody(const RobotModel &robot_model,
                         const Eigen::VectorXd &q,
                         JointPositionDof &q_ndof,
                         const MotionGroup &motion_group) {
    Eigen::VectorXd joints_map;
    switch (motion_group) {
        case MotionGroup::Primary: {
            joints_map = robot_model.params_ptr->joints_primary;
            break;
        }
        case MotionGroup::Secondary: {
            joints_map = robot_model.params_ptr->joints_secondary;
            break;
        }
        case MotionGroup::BothArms: {
            joints_map.resize(robot_model.params_ptr->joints_primary.size() +
                              robot_model.params_ptr->joints_secondary.size());
            joints_map << robot_model.params_ptr->joints_primary, robot_model.params_ptr->joints_secondary;
            break;
        }
        case MotionGroup::WholeBody: {
            joints_map = Eigen::VectorXd::LinSpaced(ndof_robot, 0, ndof_robot - 1);
            break;
        }
        case MotionGroup::Torso: {
            joints_map = robot_model.params_ptr->joints_torso;
            break;
        }
        case MotionGroup::Unknown:
        default:
            assert(false && "Unknown MotionGroup type");
            break;
    }
    // 把target_q按照joints_map映射到target_ndof
    assert(q.size() == joints_map.size() && "Target q size must match joints map size");
    for (auto i = 0; i < joints_map.size(); ++i) {
        int joint_id = static_cast<int>(joints_map[i]);
        if (joint_id >= 0 && joint_id < q_ndof.size()) {
            q_ndof[joint_id] = q[i];
        }
    }
    return true;
}

bool GetJointFromWholeBody(const RobotModel &robot_model,
                           const JointPositionDof &q_ndof,
                           Eigen::VectorXd &q,
                           const MotionGroup &motion_group) {
    Eigen::VectorXd joints_map;
    switch (motion_group) {
        case MotionGroup::Primary: {
            joints_map = robot_model.params_ptr->joints_primary;
            break;
        }
        case MotionGroup::Secondary: {
            joints_map = robot_model.params_ptr->joints_secondary;
            break;
        }
        case MotionGroup::BothArms: {
            joints_map.resize(robot_model.params_ptr->joints_primary.size() +
                              robot_model.params_ptr->joints_secondary.size());
            joints_map << robot_model.params_ptr->joints_primary, robot_model.params_ptr->joints_secondary;
            break;
        }
        case MotionGroup::WholeBody: {
            joints_map = Eigen::VectorXd::LinSpaced(ndof_robot, 0, ndof_robot - 1);
            break;
        }
        case MotionGroup::Torso: {
            joints_map = robot_model.params_ptr->joints_torso;
            break;
        }
        case MotionGroup::Unknown:
        default:
            assert(false && "Unknown MotionGroup type");
            break;
    }
    // 把target_ndof按照joints_map映射到target_q
    q = Eigen::VectorXd::Zero(joints_map.size());
    for (auto i = 0; i < joints_map.size(); ++i) {
        int joint_id = static_cast<int>(joints_map[i]);
        if (joint_id >= 0 && joint_id < q_ndof.size()) {
            q[i] = q_ndof[joint_id];
        }
    }
    return true;
}

PlannerRequest ParseSetPropertyRequest(
    const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request) {
    PlannerRequest planner_request;

    const auto request_type = magic_enum::enum_cast<RequestType>(request->property_type);
    if (!request_type.has_value()) {
        planner_request.type = RequestType::Empty;
        planner_request.data = std::nullopt;
        LOG_CRIT << "Parse property: type not defined: " << request->property_type;
        std::cout << "[Error] Parse property: type not defined: " << request->property_type << "\n";
        return planner_request;
    }

    auto parse_bool = [](const std::string &v) { return v == "1" || v == "true" || v == "True"; };
    auto parse_int = [](const std::string &v) { return std::stoi(v); };

    planner_request.type = *request_type;
    switch (planner_request.type) {
        case RequestType::TcpPrimary:
        case RequestType::TcpSecondary:
            planner_request.data = utils::StringToPoseQuaternion(request->value);
            break;
        case RequestType::SpeedRatio:
            planner_request.data = std::clamp(std::stod(request->value), 0.01, 1.0);
            break;
        case RequestType::ClearError:
            planner_request.data = std::nullopt;
            break;
        case RequestType::SafeLevel:
            planner_request.data = std::clamp(std::stod(request->value), 0.1, 1.0);
            break;
        case RequestType::SwitchRunMode:
            planner_request.data = parse_int(request->value);
            break;
        case RequestType::EnableDrag:
        case RequestType::EnableSleep:
            planner_request.data = parse_bool(request->value);
            break;
        case RequestType::JointStiffness:
        case RequestType::JointDamping:
        case RequestType::CartStiffness:
        case RequestType::CartDampingRatio:
            planner_request.data = utils::StringToVectorXd(request->value);
            break;
        default:
            planner_request.type = RequestType::Empty;
            planner_request.data = std::nullopt;
            LOG_CRIT << "Parse property: type not supported: " << request->property_type;
            break;
    }
    return planner_request;
}

// 解析MoveRobot动作的命令
PlannerRequest ParseMoveRequest(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> action_handle) {
    const auto &input_request = action_handle->get_goal();

    PlannerRequest planner_request;

    const auto request_type = magic_enum::enum_cast<RequestType>(input_request->type);
    if (!request_type.has_value()) {
        planner_request.type = RequestType::Unknown;
        return planner_request;
    }

    planner_request.type = *request_type;
    if (planner_request.type == RequestType::StopMove)  // StopMove指令无data
    {
        return planner_request;
    }

    MoveRequest move_request;
    const auto motion_group = magic_enum::enum_cast<MotionGroup>(input_request->motion_group);
    if (!motion_group.has_value()) {
        planner_request.type = RequestType::Unknown;
        LOG_CRIT << "Parse move request: unknown motion group: " << input_request->motion_group;
        std::cout << "[Error] Parse move request: unknown motion group: " << input_request->motion_group << "\n";
    }
    move_request.motion_group = *motion_group;

    // 解析目标位置 Parse target position
    if (input_request->target.size() > 0) {
        Eigen::VectorXd value = Eigen::VectorXd::Zero(input_request->target.size());
        for (size_t i = 0; i < input_request->target.size(); ++i) {
            value[i] = input_request->target[i];
        }
        move_request.value = value;
    }

    // 解析配置参数 Parse configuration parameters
    if (!input_request->config.empty()) {
        const auto input_config_json = json::parse(input_request->config);

        auto set_double = [&](const char *key, std::optional<double> &dst) {
            if (input_config_json.contains(key)) {
                dst.emplace(input_config_json[key].get<double>());
                LOG_INFO << "Parsed config: " << key << " = " << dst.value();
            }
        };
        auto set_bool = [&](const char *key, std::optional<bool> &dst) {
            if (input_config_json.contains(key)) {
                dst.emplace(input_config_json[key].get<bool>());
                LOG_INFO << "Parsed config: " << key << " = " << (dst.value() ? "true" : "false");
            }
        };
        set_bool("use_ruckig", move_request.use_ruckig);
        set_bool("use_filter", move_request.use_filter);
        set_double("speed", move_request.speed);
        set_double("acceleration", move_request.acceleration);
        set_double("blend_tolerance", move_request.blend_tolerance);
        set_double("interval", move_request.interval);
        if (input_config_json.contains("reference_frame")) {
            const auto ref =
                magic_enum::enum_cast<ReferenceFrame>(input_config_json["reference_frame"].get<std::string>());
            move_request.reference_frame.emplace(ref.value_or(ReferenceFrame::Unknown));
            LOG_INFO << "Parsed config: reference_frame = " << input_config_json["reference_frame"].get<std::string>();
        }
        if (input_config_json.contains("endpoint_index")) {
            std::vector<int> endpoint_index;
            for (const auto &item : input_config_json["endpoint_index"]) endpoint_index.push_back(item.get<int>());
            move_request.endpoint_index.emplace(std::move(endpoint_index));
            LOG_INFO << "Parsed config: endpoint_index = " << move_request.endpoint_index.value().size() << " points";
        }
    }
    planner_request.data = move_request;
    return planner_request;
}

StreamingCommand ParseStreamingCommand(const std::shared_ptr<robot_server_interfaces::msg::StreamingCommand> msg) {
    StreamingCommand streaming_command;

    streaming_command.request_id = msg->request_id;
    streaming_command.streaming_finished = msg->streaming_finished;
    streaming_command.received_time = TimePointSystem::Now();
    if (msg->position.size() > 0) {
        streaming_command.command.resize(msg->position.size());
        for (size_t i = 0; i < msg->position.size(); ++i) {
            streaming_command.command[i] = msg->position[i];
        }
    }
    return streaming_command;
}

Vector12d GetSensorWrench(const Vector6d &wrench_primary, const Vector6d &wrench_secondary) {
    // 最小力/力矩阈值(去除摩擦力), 和最大力/力矩值
    constexpr double force_threshold = 2;
    constexpr double torque_threshold = 0.1;
    constexpr double force_max = 80.0;
    constexpr double torque_max = 4.0;

    auto deadzone_saturate = [](double x, double th, double mx) noexcept -> double {
        double y = std::max(0.0, std::abs(x) - th);
        if (y > mx) y = mx;
        return std::copysign(y, x);
    };

    auto process6 = [&](const Vector6d &w) noexcept -> Vector6d {
        Vector6d out;
        for (int i = 0; i < 6; ++i) {
            const bool is_force = i < 3;
            const double th = is_force ? force_threshold : torque_threshold;
            const double mx = is_force ? force_max : torque_max;
            out[i] = deadzone_saturate(w[i], th, mx);
        }
        return out;
    };

    Vector12d sensor_wrench;
    sensor_wrench.head<6>() = process6(wrench_primary);
    sensor_wrench.tail<6>() = process6(wrench_secondary);
    return sensor_wrench;
}

// 线性插值器实现：用于在高低频率控制线程间平滑过渡
LinearInterpolator::LinearInterpolator(int interpolation_steps) : interpolation_steps_(interpolation_steps) {
    assert(interpolation_steps_ > 0 && "interpolation_steps must be > 0");
    pre_target_position_.setZero();
}

VectorDof LinearInterpolator::Interpolate(const VectorDof &target_position) {
    if (!inited_) {
        // 首次调用，初始化
        counter_ = 0;
        inited_ = true;
        target_position_ = target_position;
        pre_target_position_ = target_position;
        return target_position_;
    }
    // 如果目标位置发生变化，更新起始位置并重置计数器
    if (TargetChanged(target_position)) {
        counter_ = 0;
        pre_target_position_ = target_position_;
        target_position_ = target_position;
    }

    if (counter_ < interpolation_steps_) {
        // 线性插值: alpha 从 1/interpolation_steps_ 到 1.0
        double alpha = static_cast<double>(counter_ + 1) / interpolation_steps_;
        VectorDof result = pre_target_position_ + (target_position - pre_target_position_) * alpha;
        counter_++;
        return result;
    } else {
        // 如果没变化且插值周期完成，更新起始位置为当前目标位置
        counter_ = interpolation_steps_;
        pre_target_position_ = target_position_;
        return pre_target_position_;
    }
}

void LinearInterpolator::Reset() {
    if (!inited_) return;
    pre_target_position_.setZero();
    target_position_.setZero();
    counter_ = 0;
    inited_ = false;
}

}  // namespace utils
}  // namespace robot_server_lib
