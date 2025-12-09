#include "utils.h"

namespace robot_server_lib
{
namespace utils
{

RobotModel GetRobotModelFromConfig(std::filesystem::path robot_cfg_path)
{
    json config_json;
    RobotParams params;
    RobotModel robot_model;
    if (!std::filesystem::exists(robot_cfg_path))
    {
        LOG_CRIT << "Robot configuration file not found: " << robot_cfg_path;
        assert(false && "Robot configuration file not found");
    }
    std::ifstream ifs(robot_cfg_path);
    if (!ifs.is_open() || !(ifs >> config_json))
    {
        LOG_CRIT << "Can't open or parse robot.cfg: " << robot_cfg_path;
        assert(false && "Can't open or parse robot.cfg");
    }
    const auto package_share_path = ament_index_cpp::get_package_share_directory("robot_server_lib");
    if (!config_json.contains("robot_name") || !config_json["robot_name"].is_string())
    {
        LOG_CRIT << "robot.cfg missing 'robot_name' field";
        assert(false && "robot.cfg missing 'robot_name' field");
    }
    params.robot_name = config_json["robot_name"].get<std::string>();
    params.urdf_path = std::filesystem::path(package_share_path) / "robot_models" / params.robot_name / (params.robot_name + ".urdf");

    if (config_json.contains("left_arm_name"))
    {
        params.left_arm_name = config_json["left_arm_name"].get<std::string>();
        params.left_arm_urdf_path =
            std::filesystem::path(package_share_path) / "robot_models" / params.left_arm_name / (params.left_arm_name + ".urdf");
    }
    if (config_json.contains("right_arm_name"))
    {
        params.right_arm_name = config_json["right_arm_name"].get<std::string>();
        params.right_arm_urdf_path =
            std::filesystem::path(package_share_path) / "robot_models" / params.right_arm_name / (params.right_arm_name + ".urdf");
    }

    auto parse_array = [&](const std::string &key, Eigen::VectorXd &dst)
    {
        if (config_json.contains(key))
        {
            auto arr = config_json[key];
            dst = Eigen::VectorXd::Zero(arr.size());
            for (size_t i = 0; i < arr.size(); ++i)
            {
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
    if (config_json.contains("dyn_base_params_primary"))
    {
        Eigen::VectorXd dyn_base_params_primary;
        parse_array("dyn_base_params_primary", dyn_base_params_primary);
        params.dyn_base_params_primary = dyn_base_params_primary;
        LOG_INFO << "Loaded dyn_base_params_primary with size: " << dyn_base_params_primary.size();
    }
    if (config_json.contains("dyn_base_params_secondary"))
    {
        Eigen::VectorXd dyn_base_params_secondary;
        parse_array("dyn_base_params_secondary", dyn_base_params_secondary);
        params.dyn_base_params_secondary = dyn_base_params_secondary;
        LOG_INFO << "Loaded dyn_base_params_secondary with size: " << dyn_base_params_secondary.size();
    }

    if (config_json.contains("joint_command_sequence_by_name"))
    {
        auto arr = config_json["joint_command_sequence_by_name"];
        params.joint_command_sequence_by_name.clear();
        for (const auto &item : arr)
        {
            params.joint_command_sequence_by_name.push_back(item.get<std::string>());
        }
    }

    pinocchio::Model model, left_arm_model, right_arm_model;
    pinocchio::urdf::buildModel(params.urdf_path, model);
    pinocchio::urdf::buildModel(params.left_arm_urdf_path, left_arm_model);
    pinocchio::urdf::buildModel(params.right_arm_urdf_path, right_arm_model);

    robot_model.model_ptr = std::make_shared<pinocchio::Model>(model);
    robot_model.left_arm_model_ptr = std::make_shared<pinocchio::Model>(left_arm_model);
    robot_model.right_arm_model_ptr = std::make_shared<pinocchio::Model>(right_arm_model);
    robot_model.params_ptr = std::make_shared<RobotParams>(params);

    return robot_model;
}

void SetMax(std::atomic_int64_t &maximum_value, std::int64_t value)
{
    std::int64_t prev_value = maximum_value;
    while (prev_value < value && !maximum_value.compare_exchange_weak(prev_value, value))
    {
    }
}

PoseQuaternion VectorXdToPoseQuaternion(const Eigen::VectorXd &vec)
{
    PoseQuaternion pose;
    pose.position = Eigen::Vector3d(vec[0], vec[1], vec[2]);
    pose.orientation = Eigen::Quaterniond(vec[3], vec[4], vec[5], vec[6]);
    pose.orientation.normalize();
    return pose;
}

PoseQuaternion StringToPoseQuaternion(const std::string &str)
{
    PoseQuaternion pose;
    std::stringstream ss(str);
    std::string item;
    double values[7] = {0};
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 7)
    {
        values[idx++] = std::stod(item);
    }
    pose.position = Eigen::Vector3d(values[0], values[1], values[2]);
    pose.orientation = Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
    pose.orientation.normalize();
    return pose;
}

Vector3d StringToVector3d(const std::string &str)
{
    std::stringstream ss(str);
    std::string item;
    Vector3d values;
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 3)
    {
        values[idx++] = std::stod(item);
    }
    return values;
}

bool MapJointToWholeBody(const RobotModel &robot_model, const Eigen::VectorXd &q, JointPositionDof &q_ndof, const MotionGroup &motion_group)
{
    Eigen::VectorXd joints_map;
    switch (motion_group)
    {
    case MotionGroup::Primary:
    {
        joints_map = robot_model.params_ptr->joints_primary;
        break;
    }
    case MotionGroup::Secondary:
    {
        joints_map = robot_model.params_ptr->joints_secondary;
        break;
    }
    case MotionGroup::BothArms:
    {
        joints_map.resize(robot_model.params_ptr->joints_primary.size() + robot_model.params_ptr->joints_secondary.size());
        joints_map << robot_model.params_ptr->joints_primary, robot_model.params_ptr->joints_secondary;
        break;
    }
    case MotionGroup::WholeBody:
    {
        joints_map = Eigen::VectorXd::LinSpaced(ndof_robot, 0, ndof_robot - 1);
        break;
    }
    case MotionGroup::Torso:
    {
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
    for (auto i = 0; i < joints_map.size(); ++i)
    {
        int joint_id = static_cast<int>(joints_map[i]);
        if (joint_id >= 0 && joint_id < q_ndof.size())
        {
            q_ndof[joint_id] = q[i];
        }
    }
    return true;
}

bool GetJointFromWholeBody(const RobotModel &robot_model, const JointPositionDof &q_ndof, Eigen::VectorXd &q, const MotionGroup &motion_group)
{
    Eigen::VectorXd joints_map;
    switch (motion_group)
    {
    case MotionGroup::Primary:
    {
        joints_map = robot_model.params_ptr->joints_primary;
        break;
    }
    case MotionGroup::Secondary:
    {
        joints_map = robot_model.params_ptr->joints_secondary;
        break;
    }
    case MotionGroup::BothArms:
    {
        joints_map.resize(robot_model.params_ptr->joints_primary.size() + robot_model.params_ptr->joints_secondary.size());
        joints_map << robot_model.params_ptr->joints_primary, robot_model.params_ptr->joints_secondary;
        break;
    }
    case MotionGroup::WholeBody:
    {
        joints_map = Eigen::VectorXd::LinSpaced(ndof_robot, 0, ndof_robot - 1);
        break;
    }
    case MotionGroup::Torso:
    {
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
    for (auto i = 0; i < joints_map.size(); ++i)
    {
        int joint_id = static_cast<int>(joints_map[i]);
        if (joint_id >= 0 && joint_id < q_ndof.size())
        {
            q[i] = q_ndof[joint_id];
        }
    }
    return true;
}

PlannerRequest ParseSetPropertyRequest(const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request)
{
    PlannerRequest planner_request;
    // 解析请求类型 Parse request type
    if (request->property_type == "TcpPrimary")
    {
        planner_request.type = RequestType::TcpPrimary;
        planner_request.data = utils::StringToPoseQuaternion(request->value);
    }
    else if (request->property_type == "TcpSecondary")
    {
        planner_request.type = RequestType::TcpSecondary;
        planner_request.data = utils::StringToPoseQuaternion(request->value);
    }
    else if (request->property_type == "SpeedRatio")
    {
        planner_request.type = RequestType::SpeedRatio;
        double speed = std::clamp(std::stod(request->value), 0.01, 1.0);
        planner_request.data = speed;
    }
    else if (request->property_type == "ClearError")
    {
        planner_request.type = RequestType::ClearError;
        planner_request.data = std::nullopt;  // Clear error request does not require additional data
    }
    else if (request->property_type == "SafeLevel")
    {
        planner_request.type = RequestType::SafeLevel;
        planner_request.data = std::clamp(std::stod(request->value), 0.1, 1.0);
    }
    else if (request->property_type == "EnableDrag")
    {
        planner_request.type = RequestType::EnableDrag;
        planner_request.data = request->value == "1" ? true : false;
    }
    else if (request->property_type == "EnableImpedance")
    {
        planner_request.type = RequestType::EnableImpedance;
        planner_request.data = request->value == "1" ? true : false;
    }
    else if (request->property_type == "ImpedanceMass")
    {
        planner_request.type = RequestType::ImpedanceMass;
        planner_request.data = utils::StringToVector3d(request->value);
    }
    else if (request->property_type == "ImpedanceDamping")
    {
        planner_request.type = RequestType::ImpedanceDamping;
        planner_request.data = utils::StringToVector3d(request->value);
    }
    else if (request->property_type == "ImpedanceStiffness")
    {
        planner_request.type = RequestType::ImpedanceStiffness;
        planner_request.data = utils::StringToVector3d(request->value);
    }
    else
    {
        planner_request.type = RequestType::Empty;
        LOG_CRIT << "Parse property: type not defined: " << request->property_type;
    }
    return planner_request;
}

// 解析MoveRobot动作的命令
PlannerRequest ParseMoveRequest(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> action_handle)
{
    const auto &input_request = action_handle->get_goal();

    PlannerRequest planner_request;
    // 解析请求类型 Parse request type
    if (input_request->type == "MoveJoint")
    {
        planner_request.type = RequestType::MoveJoint;
    }
    else if (input_request->type == "MoveLine")
    {
        planner_request.type = RequestType::MoveLine;
    }
    else if (input_request->type == "MoveTraj")
    {
        planner_request.type = RequestType::MoveTraj;
    }
    else if (input_request->type == "StartJointStreaming")
    {
        planner_request.type = RequestType::StartJointStreaming;
    }
    else if (input_request->type == "StartEEPoseStreaming")
    {
        planner_request.type = RequestType::StartEEPoseStreaming;
    }
    else if (input_request->type == "StopMove")
    {
        planner_request.type = RequestType::StopMove;
        return planner_request;
    }
    else
    {
        planner_request.type = RequestType::Unknown;
        return planner_request;
    }

    MoveRequest move_request;
    // 解析motion_group Parse motion group
    if (input_request->motion_group == "Primary")
    {
        move_request.motion_group = MotionGroup::Primary;
    }
    else if (input_request->motion_group == "Secondary")
    {
        move_request.motion_group = MotionGroup::Secondary;
    }
    else if (input_request->motion_group == "BothArms")
    {
        move_request.motion_group = MotionGroup::BothArms;
    }
    else if (input_request->motion_group == "WholeBody")
    {
        move_request.motion_group = MotionGroup::WholeBody;
    }
    else if (input_request->motion_group == "Torso")
    {
        move_request.motion_group = MotionGroup::Torso;
    }
    else
    {
        move_request.motion_group = MotionGroup::Unknown;
    }

    // 解析目标位置 Parse target position
    Eigen::VectorXd value;
    if (input_request->target.size() > 0)
    {
        value.resize(input_request->target.size());
        for (size_t i = 0; i < input_request->target.size(); ++i)
        {
            value[i] = input_request->target[i];
        }
        move_request.value = value;
    }

    // 解析配置参数 Parse configuration parameters
    json input_config_json;
    if (!input_request->config.empty())
    {
        input_config_json = json::parse(input_request->config);

        if (input_config_json.contains("reference_frame"))
        {
            if (input_request->motion_group == "ArmBase")
            {
                move_request.reference_frame.emplace(ReferenceFrame::ArmBase);
            }
            else if (input_request->motion_group == "WorldBase")
            {
                move_request.reference_frame.emplace(ReferenceFrame::WorldBase);
            }
            else
            {
                move_request.reference_frame.emplace(ReferenceFrame::Unknown);
            }
        }

        if (input_config_json.contains("speed"))
        {
            move_request.speed.emplace(input_config_json["speed"].get<double>());
        }
        if (input_config_json.contains("blend_tolerance"))
        {
            move_request.blend_tolerance.emplace(input_config_json["blend_tolerance"].get<double>());
        }
        if (input_config_json.contains("optional_mode"))
        {
            move_request.optional_mode.emplace(input_config_json["optional_mode"].get<bool>());
        }
        if (input_config_json.contains("interval"))
        {
            move_request.interval.emplace(input_config_json["interval"].get<double>());
        }
        if (input_config_json.contains("endpoint_index"))
        {
            std::vector<int> endpoint_index;
            for (const auto &item : input_config_json["endpoint_index"])
            {
                endpoint_index.push_back(item.get<int>());
            }
            move_request.endpoint_index.emplace(endpoint_index);
        }
    }
    planner_request.data = move_request;
    return planner_request;
}

StreamingCommand ParseStreamingCommand(const std::shared_ptr<robot_server_interfaces::msg::StreamingCommand> msg)
{
    StreamingCommand streaming_command;

    streaming_command.request_id = msg->request_id;
    streaming_command.streaming_finished = msg->streaming_finished;
    streaming_command.received_time = TimePointSystem::Now();
    if (msg->position.size() > 0)
    {
        streaming_command.command.resize(msg->position.size());
        for (size_t i = 0; i < msg->position.size(); ++i)
        {
            streaming_command.command[i] = msg->position[i];
        }
    }
    return streaming_command;
}

Vector12d GetSensorWrench(const Vector6d &wrench_primary, const Vector6d &wrench_secondary)
{
    // 最小力/力矩阈值(去除摩擦力), 和最大力/力矩值
    constexpr double force_threshold = 2;
    constexpr double torque_threshold = 0.1;
    constexpr double force_max = 80.0;
    constexpr double torque_max = 4.0;

    auto deadzone_saturate = [](double x, double th, double mx) noexcept -> double
    {
        double y = std::max(0.0, std::abs(x) - th);
        if (y > mx)
            y = mx;
        return std::copysign(y, x);
    };

    auto process6 = [&](const Vector6d &w) noexcept -> Vector6d
    {
        Vector6d out;
        for (int i = 0; i < 6; ++i)
        {
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

}  // namespace utils
}  // namespace robot_server_lib
