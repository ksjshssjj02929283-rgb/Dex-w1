#include "utils.h"

namespace robot_server_lib {
namespace utils {

RobotModel GetRobotModelFromConfig(std::filesystem::path robot_cfg_path) {
    YAML::Node config;
    RobotParams params;
    RobotModel robot_model;
    if (!std::filesystem::exists(robot_cfg_path)) {
        LOG_CRIT << "Robot configuration file not found: " << robot_cfg_path;
        std::cout << "[Error] Robot configuration file not found: " << robot_cfg_path << "\n";
        exit(EXIT_FAILURE);
    }
    try {
        config = YAML::LoadFile(robot_cfg_path.string());
    } catch (const YAML::Exception& e) {
        LOG_CRIT << "Can't parse robot_config.yaml: " << e.what();
        std::cout << "[Error] Can't parse robot_config.yaml: " << e.what() << "\n";
        exit(EXIT_FAILURE);
    }
    if (config["robot_type"]) {
        params.robot_type = config["robot_type"].as<std::string>();
    } else {
        LOG_CRIT << "robot_config.yaml missing 'robot_type' field";
        std::cout << "[Error] robot_config.yaml missing 'robot_type' field\n";
        exit(EXIT_FAILURE);
    }

    if (config["version"]) {
        params.robot_version = config["version"].as<std::string>();
    } else {
        LOG_CRIT << "robot_config.yaml missing 'version' field";
        std::cout << "[Error] robot_config.yaml missing 'version' field\n";
        exit(EXIT_FAILURE);
    }

    YAML::Node version_cfg;
    if (params.robot_version.size() >= 3) {
        std::string version_prefix = params.robot_version.substr(0, 3);
        std::string full_version_key = params.robot_type + "_" + version_prefix;
        params.robot_key = full_version_key;
        if (config[full_version_key]) {
            version_cfg = config[full_version_key];
            std::cout << "[INFO] Robot type: " << params.robot_type << ", Robot version: " << params.robot_version
                      << ", Using version config: " << full_version_key << "\n";
            LOG_INFO << "Robot type: " << params.robot_type << ", Robot version: " << params.robot_version
                     << ", Using version config: " << full_version_key;
        } else {
            LOG_CRIT << "Version config " << full_version_key << " not found in config file";
            std::cout << "[Error] Version config " << full_version_key << " not found in config file\n";
            exit(EXIT_FAILURE);
        }
    } else {
        LOG_CRIT << "Invalid robot version format: " << params.robot_version;
        std::cout << "[Error] Invalid robot version format: " << params.robot_version << "\n";
        exit(EXIT_FAILURE);
    }

    if (config["left_hand_type"]) {
        params.left_hand_type = config["left_hand_type"].as<std::string>();
    }
    if (config["right_hand_type"]) {
        params.right_hand_type = config["right_hand_type"].as<std::string>();
    }

    if (config["ros2_to_urdf_map"] && config["ros2_to_urdf_map"].IsMap()) {
        for (auto it = config["ros2_to_urdf_map"].begin(); it != config["ros2_to_urdf_map"].end(); ++it) {
            params.ros2_to_urdf_map[it->first.as<std::string>()] = it->second.as<std::string>();
        }
    }

    auto parse_string_list = [&](const char* key, std::vector<std::string>& dst) {
        if (!config[key]) {
            return;
        }
        if (config[key].IsSequence()) {
            for (const auto& item : config[key]) {
                dst.push_back(item.as<std::string>());
            }
            return;
        }
        if (config[key].IsScalar()) {
            dst.push_back(config[key].as<std::string>());
        }
    };

    parse_string_list("joints_primary_names", params.joints_primary_names);
    parse_string_list("joints_secondary_names", params.joints_secondary_names);
    parse_string_list("joints_torso_names", params.joints_torso_names);
    parse_string_list("joints_a1", params.joints_a1);
    parse_string_list("joints_single_arm", params.joints_single_arm);
    parse_string_list("joints_dual_arm", params.joints_dual_arm);
    if (config["ee_primary_name"]) {
        params.ee_primary_name = config["ee_primary_name"].as<std::string>();
    }
    if (config["ee_secondary_name"]) {
        params.ee_secondary_name = config["ee_secondary_name"].as<std::string>();
    }

    std::filesystem::path robot_urdf_path;
    std::filesystem::path primary_arm_urdf_path;
    std::filesystem::path secondary_arm_urdf_path;

    if (config["file_path"] && config["file_path"]["robot_urdf"]) {
        robot_urdf_path = config["file_path"]["robot_urdf"].as<std::string>();
        if (!std::filesystem::exists(robot_urdf_path)) {
            LOG_CRIT << "Robot URDF file not found: " << robot_urdf_path;
            std::cout << "[Error] Robot URDF file not found: " << robot_urdf_path << "\n";
            exit(EXIT_FAILURE);
        }
        primary_arm_urdf_path = robot_urdf_path.parent_path() / "left_arm.urdf";
        secondary_arm_urdf_path = robot_urdf_path.parent_path() / "right_arm.urdf";

        if (!std::filesystem::exists(primary_arm_urdf_path)) {
            LOG_CRIT << "Primary arm URDF file not found: " << primary_arm_urdf_path;
            std::cout << "[Error] Primary arm URDF file not found: " << primary_arm_urdf_path << "\n";
            exit(EXIT_FAILURE);
        }
        if (!std::filesystem::exists(secondary_arm_urdf_path)) {
            LOG_CRIT << "Secondary arm URDF file not found: " << secondary_arm_urdf_path;
            std::cout << "[Error] Secondary arm URDF file not found: " << secondary_arm_urdf_path << "\n";
            exit(EXIT_FAILURE);
        }
        std::cout << "[INFO] Robot_urdf_path: " << robot_urdf_path << "\n";
    } else {
        LOG_CRIT << "Robot_config.yaml missing 'file_path.robot_urdf' field";
        std::cout << "[Error] robot_config.yaml missing 'file_path.robot_urdf' field\n";
        exit(EXIT_FAILURE);
    }
    pinocchio::Model model, left_arm_model, right_arm_model;
    pinocchio::urdf::buildModel(robot_urdf_path, model);
    pinocchio::urdf::buildModel(primary_arm_urdf_path, left_arm_model);
    pinocchio::urdf::buildModel(secondary_arm_urdf_path, right_arm_model);

    if (version_cfg["dyn_base_params_primary"]) {
        auto parse_array = [&](const std::string& key, Eigen::VectorXd& dst) {
            if (version_cfg[key] && version_cfg[key].IsSequence()) {
                const auto size = version_cfg[key].size();
                dst = Eigen::VectorXd::Zero(size);
                for (size_t i = 0; i < size; ++i) {
                    dst[i] = version_cfg[key][i].as<double>();
                }
            }
        };
        Eigen::VectorXd dyn_base_params_primary;
        parse_array("dyn_base_params_primary", dyn_base_params_primary);
        params.dyn_base_params_primary = dyn_base_params_primary;
    } else {
        std::cout << "Use dafault dynamic model in urdf\n";
        LOG_INFO << "Use dafault dynamic model in urdf";
    }
    if (version_cfg["dyn_base_params_secondary"]) {
        auto parse_array = [&](const std::string& key, Eigen::VectorXd& dst) {
            if (version_cfg[key] && version_cfg[key].IsSequence()) {
                const auto size = version_cfg[key].size();
                dst = Eigen::VectorXd::Zero(size);
                for (size_t i = 0; i < size; ++i) {
                    dst[i] = version_cfg[key][i].as<double>();
                }
            }
        };
        Eigen::VectorXd dyn_base_params_secondary;
        parse_array("dyn_base_params_secondary", dyn_base_params_secondary);
        params.dyn_base_params_secondary = dyn_base_params_secondary;
    } else {
        std::cout << "Use dafault dynamic model in urdf\n";
        LOG_INFO << "Use dafault dynamic model in urdf";
    }

    robot_model.model_ptr = std::make_shared<pinocchio::Model>(model);
    robot_model.primary_arm_model_ptr = std::make_shared<pinocchio::Model>(left_arm_model);
    robot_model.secondary_arm_model_ptr = std::make_shared<pinocchio::Model>(right_arm_model);

    int num_joints = model.nq;
    params.joint_velocity_limit = Eigen::VectorXd::Zero(num_joints);
    params.joint_torque_limit = Eigen::VectorXd::Zero(num_joints);
    params.joint_limit_margin_deg = Eigen::VectorXd::Zero(num_joints);

    RobotProperty robot_property;

    if (version_cfg["allow_force_control"]) {
        robot_property.allow_force_control = version_cfg["allow_force_control"].as<bool>();
    }

    if (version_cfg["joint_streaming_bounds"] && version_cfg["joint_streaming_bounds"].IsMap()) {
        if (version_cfg["joint_streaming_bounds"]["max_velocity"]) {
            robot_property.joint_streaming_bounds.max_velocity =
                version_cfg["joint_streaming_bounds"]["max_velocity"].as<double>();
        }
        if (version_cfg["joint_streaming_bounds"]["max_acceleration"]) {
            robot_property.joint_streaming_bounds.max_acceleration =
                version_cfg["joint_streaming_bounds"]["max_acceleration"].as<double>();
        }
        if (version_cfg["joint_streaming_bounds"]["max_jerk"]) {
            robot_property.joint_streaming_bounds.max_jerk =
                version_cfg["joint_streaming_bounds"]["max_jerk"].as<double>();
        }
    }

    if (version_cfg["enable_streaming_ruckig"]) {
        robot_property.enable_streaming_ruckig = version_cfg["enable_streaming_ruckig"].as<bool>();
    }

    if (config["enable_record"]) {
        robot_property.enable_record = config["enable_record"].as<bool>();
    }

    if (config["chassis_config"] && config["chassis_config"]["enable_chassis_shaper"]) {
        robot_property.enable_chassis_shaper = config["chassis_config"]["enable_chassis_shaper"].as<bool>();
    }

    if (version_cfg["joints"] && version_cfg["joints"].IsSequence()) {
        int joint_idx = 0;
        for (const auto& joint_config : version_cfg["joints"]) {
            std::string joint_name;
            if (joint_config["name"]) {
                joint_name = joint_config["name"].as<std::string>();
            } else {
                continue;
            }

            // 校验配置文件中的名称与URDF中的名称及顺序是否一致 (考虑映射)
            std::string urdf_name;
            if (params.ros2_to_urdf_map.count(joint_name)) {
                urdf_name = params.ros2_to_urdf_map.at(joint_name);
            }

            if (joint_idx + 1 >= static_cast<int>(model.names.size())) {
                LOG_CRIT << "Config has more joints than URDF! Surplus joint in config: " << joint_name;
                std::cout << "[Error] Config has more joints than URDF! Surplus joint in config: " << joint_name
                          << "\n";
                exit(EXIT_FAILURE);
            }

            if (model.names[joint_idx + 1] != urdf_name) {
                LOG_CRIT << "Joint name/order mismatch! Index: " << joint_idx << ", Config: " << joint_name
                         << " (mapped: " << urdf_name << "), URDF (Pinocchio): " << model.names[joint_idx + 1];
                std::cout << "[Error] Joint name/order mismatch! Index: " << joint_idx << ", Config: " << joint_name
                          << " (mapped: " << urdf_name << "), URDF (Pinocchio): " << model.names[joint_idx + 1] << "\n";
                exit(EXIT_FAILURE);
            }

            params.joint_names.push_back(joint_name);

            if (joint_config["velocity_limit"]) {
                params.joint_velocity_limit[joint_idx] = joint_config["velocity_limit"].as<double>();
            }

            if (joint_config["torque_limit"]) {
                params.joint_torque_limit[joint_idx] = joint_config["torque_limit"].as<double>();
            }

            if (joint_config["position_limit_margin_deg"]) {
                params.joint_limit_margin_deg[joint_idx] = joint_config["position_limit_margin_deg"].as<double>();
            }

            if (joint_config["joint_impedance_control"]) {
                if (joint_config["joint_impedance_control"]["stiffness"]) {
                    robot_property.joint_stiffness[joint_idx] =
                        joint_config["joint_impedance_control"]["stiffness"].as<double>();
                }
                if (joint_config["joint_impedance_control"]["damping"]) {
                    robot_property.joint_damping[joint_idx] =
                        joint_config["joint_impedance_control"]["damping"].as<double>();
                }
                if (joint_config["joint_impedance_control"]["integral_ratio"]) {
                    robot_property.joint_integral_ratio[joint_idx] =
                        joint_config["joint_impedance_control"]["integral_ratio"].as<double>();
                }
            }

            if (joint_config["butterworth_filter"]) {
                if (joint_config["butterworth_filter"]["enabled"]) {
                    robot_property.enable_butterworth_filter[joint_idx] =
                        joint_config["butterworth_filter"]["enabled"].as<bool>();
                }
                if (joint_config["butterworth_filter"]["frequency"]) {
                    robot_property.butterworth_filter_frequency[joint_idx] =
                        joint_config["butterworth_filter"]["frequency"].as<double>();
                }
            }

            if (joint_config["input_shaper"]) {
                if (joint_config["input_shaper"]["enabled"]) {
                    robot_property.enable_input_shaper[joint_idx] = joint_config["input_shaper"]["enabled"].as<bool>();
                }
                if (joint_config["input_shaper"]["frequency"]) {
                    robot_property.input_shaper_frequency[joint_idx] =
                        joint_config["input_shaper"]["frequency"].as<double>();
                }
            }
            joint_idx++;
        }
        // 校验总数是否一致
        if (joint_idx != static_cast<int>(model.names.size()) - 1) {
            LOG_CRIT << "Total joint count mismatch! Config count: " << joint_idx
                     << ", URDF count (Pinocchio): " << model.names.size() - 1;
            std::cout << "[Error] Total joint count mismatch! Config count: " << joint_idx
                      << ", URDF count (Pinocchio): " << model.names.size() - 1 << "\n";
            exit(EXIT_FAILURE);
        }
    }

    if (config["safe_level"]) {
        robot_property.safe_level = config["safe_level"].as<double>();
    }

    if (config["global_speed_ratio"]) {
        robot_property.global_speed_ratio = config["global_speed_ratio"].as<double>();
    }

    if (version_cfg["cart_stiffness"] && version_cfg["cart_stiffness"].IsSequence()) {
        const auto size = version_cfg["cart_stiffness"].size();
        robot_property.cart_stiffness = Eigen::VectorXd::Zero(size);
        for (size_t i = 0; i < size; ++i) {
            robot_property.cart_stiffness[i] = version_cfg["cart_stiffness"][i].as<double>();
        }
    }

    if (version_cfg["cart_damping_ratio"] && version_cfg["cart_damping_ratio"].IsSequence()) {
        const auto size = version_cfg["cart_damping_ratio"].size();
        robot_property.cart_damping_ratio = Eigen::VectorXd::Zero(size);
        for (size_t i = 0; i < size; ++i) {
            robot_property.cart_damping_ratio[i] = version_cfg["cart_damping_ratio"][i].as<double>();
        }
    }

    if (version_cfg["safety_check_config"] && version_cfg["safety_check_config"].IsMap()) {
        if (version_cfg["safety_check_config"]["enable_position_limit_check"]) {
            robot_property.safety_check_config.check_position_limit =
                version_cfg["safety_check_config"]["enable_position_limit_check"].as<bool>();
        }
        if (version_cfg["safety_check_config"]["enable_torque_limit_check"]) {
            robot_property.safety_check_config.check_torque_limit =
                version_cfg["safety_check_config"]["enable_torque_limit_check"].as<bool>();
        }
        if (version_cfg["safety_check_config"]["enable_velocity_limit_check"]) {
            robot_property.safety_check_config.check_velocity_limit =
                version_cfg["safety_check_config"]["enable_velocity_limit_check"].as<bool>();
        }
    }

    robot_model.params_ptr = std::make_shared<RobotParams>(params);
    robot_model.robot_property_default = robot_property;

    return robot_model;
}

void SetMax(std::atomic_int64_t& maximum_value, std::int64_t value) {
    std::int64_t prev_value = maximum_value;
    while (prev_value < value && !maximum_value.compare_exchange_weak(prev_value, value)) {
    }
}

PoseQuaternion VectorXdToPoseQuaternion(const Eigen::VectorXd& vec) {
    PoseQuaternion pose;
    pose.position = Eigen::Vector3d(vec[0], vec[1], vec[2]);
    pose.orientation = Eigen::Quaterniond(vec[3], vec[4], vec[5], vec[6]);
    pose.orientation.normalize();
    return pose;
}

PoseQuaternion StringToPoseQuaternion(const std::string& str) {
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

Vector3d StringToVector3d(const std::string& str) {
    std::stringstream ss(str);
    std::string item;
    Vector3d values;
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 3) {
        values[idx++] = std::stod(item);
    }
    return values;
}

Vector6d StringToVector6d(const std::string& str) {
    std::stringstream ss(str);
    std::string item;
    Vector6d values;
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 6) {
        values[idx++] = std::stod(item);
    }
    return values;
}

Eigen::VectorXd StringToVectorXd(const std::string& str) {
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

bool MapJointToWholeBody(const RobotModel& robot_model,
                         const Eigen::VectorXd& q,
                         JointPositionDof& q_ndof,
                         const MotionGroup& motion_group) {
    std::vector<std::string> joints_names;
    switch (motion_group) {
        case MotionGroup::Primary: {
            joints_names = robot_model.params_ptr->joints_primary_names;
            break;
        }
        case MotionGroup::Secondary: {
            joints_names = robot_model.params_ptr->joints_secondary_names;
            break;
        }
        case MotionGroup::BothArms: {
            joints_names = robot_model.params_ptr->joints_primary_names;
            joints_names.insert(joints_names.end(), robot_model.params_ptr->joints_secondary_names.begin(),
                                robot_model.params_ptr->joints_secondary_names.end());
            break;
        }
        case MotionGroup::WholeBody: {
            joints_names = robot_model.params_ptr->joint_names;
            break;
        }
        case MotionGroup::Torso: {
            joints_names = robot_model.params_ptr->joints_torso_names;
            break;
        }
        case MotionGroup::Unknown:
        default:
            LOG_CRIT << "Unknown MotionGroup type in MapJointToWholeBody";
            std::cout << "[Error] Unknown MotionGroup type in MapJointToWholeBody\n";
            exit(EXIT_FAILURE);
            break;
    }

    if (q.size() != static_cast<int>(joints_names.size())) {
        LOG_CRIT << "Target q size mismatch! Expected: " << joints_names.size() << ", Actual: " << q.size();
        std::cout << "[Error] Target q size mismatch! Expected: " << joints_names.size() << ", Actual: " << q.size()
                  << "\n";
        exit(EXIT_FAILURE);
    }

    for (size_t i = 0; i < joints_names.size(); ++i) {
        auto it = std::find(robot_model.params_ptr->joint_names.begin(), robot_model.params_ptr->joint_names.end(),
                            joints_names[i]);
        if (it != robot_model.params_ptr->joint_names.end()) {
            int joint_id = std::distance(robot_model.params_ptr->joint_names.begin(), it);
            if (joint_id >= 0 && joint_id < q_ndof.size()) {
                q_ndof[joint_id] = q[i];
            }
        }
    }
    return true;
}

bool GetJointFromWholeBody(const RobotModel& robot_model,
                           const JointPositionDof& q_ndof,
                           Eigen::VectorXd& q,
                           const MotionGroup& motion_group) {
    std::vector<std::string> joints_names;
    switch (motion_group) {
        case MotionGroup::Primary: {
            joints_names = robot_model.params_ptr->joints_primary_names;
            break;
        }
        case MotionGroup::Secondary: {
            joints_names = robot_model.params_ptr->joints_secondary_names;
            break;
        }
        case MotionGroup::BothArms: {
            joints_names = robot_model.params_ptr->joints_primary_names;
            joints_names.insert(joints_names.end(), robot_model.params_ptr->joints_secondary_names.begin(),
                                robot_model.params_ptr->joints_secondary_names.end());
            break;
        }
        case MotionGroup::WholeBody: {
            joints_names = robot_model.params_ptr->joint_names;
            break;
        }
        case MotionGroup::Torso: {
            joints_names = robot_model.params_ptr->joints_torso_names;
            break;
        }
        case MotionGroup::Unknown:
        default:
            LOG_CRIT << "Unknown MotionGroup type in GetJointFromWholeBody";
            std::cout << "[Error] Unknown MotionGroup type in GetJointFromWholeBody\n";
            exit(EXIT_FAILURE);
            break;
    }

    q = Eigen::VectorXd::Zero(joints_names.size());

    for (size_t i = 0; i < joints_names.size(); ++i) {
        auto it = std::find(robot_model.params_ptr->joint_names.begin(), robot_model.params_ptr->joint_names.end(),
                            joints_names[i]);
        if (it != robot_model.params_ptr->joint_names.end()) {
            int joint_id = std::distance(robot_model.params_ptr->joint_names.begin(), it);
            if (joint_id >= 0 && joint_id < q_ndof.size()) {
                q[i] = q_ndof[joint_id];
            }
        }
    }
    return true;
}

int GetJointCount(const RobotModel& robot_model, const MotionGroup& motion_group) {
    switch (motion_group) {
        case MotionGroup::Primary:
            return robot_model.params_ptr->joints_primary_names.size();
        case MotionGroup::Secondary:
            return robot_model.params_ptr->joints_secondary_names.size();
        case MotionGroup::BothArms:
            return robot_model.params_ptr->joints_primary_names.size() +
                   robot_model.params_ptr->joints_secondary_names.size();
        case MotionGroup::WholeBody:
            return ndof_robot;
        case MotionGroup::Torso:
            return robot_model.params_ptr->joints_torso_names.size();
        default:
            return ndof_robot;
    }
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

    auto parse_bool = [](const std::string& v) { return v == "1" || v == "true" || v == "True"; };
    auto parse_int = [](const std::string& v) { return std::stoi(v); };

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
        case RequestType::QuickStop:
        case RequestType::EnablePosLimitCheck:
        case RequestType::EnableStreamingRuckig:
        case RequestType::EnableRecord:
            planner_request.data = parse_bool(request->value);
            break;
        case RequestType::JointStiffness:
        case RequestType::JointDamping:
        case RequestType::CartStiffness:
        case RequestType::CartDampingRatio:
        case RequestType::JointStreamingBounds:
        case RequestType::EnableInputShaper:
        case RequestType::EnableButterworthFilter:
            planner_request.data = utils::StringToVectorXd(request->value);
            break;
        case RequestType::JointIntegralRatio:
            planner_request.data = std::clamp(std::stod(request->value), 0.00, 1.0);
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
    const auto& input_request = action_handle->get_goal();

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

    // 解析配置参数 Parse configuration parameters (运行时字符串，保持JSON格式)
    if (!input_request->config.empty()) {
        const auto input_config_json = nlohmann::json::parse(input_request->config);

        auto set_double = [&](const char* key, std::optional<double>& dst) {
            if (input_config_json.contains(key)) {
                dst.emplace(input_config_json[key].get<double>());
                LOG_INFO << "Parsed config: " << key << " = " << dst.value();
            }
        };
        auto set_bool = [&](const char* key, std::optional<bool>& dst) {
            if (input_config_json.contains(key)) {
                dst.emplace(input_config_json[key].get<bool>());
                LOG_INFO << "Parsed config: " << key << " = " << (dst.value() ? "true" : "false");
            }
        };
        set_bool("use_velocity", move_request.use_velocity);
        set_bool("use_acceleration", move_request.use_acceleration);
        set_double("speed", move_request.speed);
        set_double("acceleration", move_request.acceleration);
        set_double("interval", move_request.interval);
        if (input_config_json.contains("reference_frame")) {
            const auto ref =
                magic_enum::enum_cast<ReferenceFrame>(input_config_json["reference_frame"].get<std::string>());
            move_request.reference_frame.emplace(ref.value_or(ReferenceFrame::Unknown));
            LOG_INFO << "Parsed config: reference_frame = " << input_config_json["reference_frame"].get<std::string>();
        }
        if (input_config_json.contains("endpoint_index")) {
            std::vector<int> endpoint_index;
            for (const auto& item : input_config_json["endpoint_index"]) endpoint_index.push_back(item.get<int>());
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

Vector12d GetSensorWrench(const Vector6d& wrench_primary, const Vector6d& wrench_secondary) {
    // 最小力/力矩阈值(去除摩擦力), 和最大力/力矩值
    constexpr double force_threshold = 1.0;
    constexpr double torque_threshold = 0.1;
    constexpr double force_max = 100.0;
    constexpr double torque_max = 10.0;

    auto deadzone_saturate = [](double x, double th, double mx) noexcept -> double {
        double y = std::max(0.0, std::abs(x) - th);
        if (y > mx) y = mx;
        return std::copysign(y, x);
    };

    auto process6 = [&](const Vector6d& w) noexcept -> Vector6d {
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

VectorDof LinearInterpolator::Interpolate(const VectorDof& target_position) {
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
