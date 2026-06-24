#include "utils.h"

#include <cstdlib>

namespace robot_server_lib {
namespace utils {

namespace {
// 递归合并用户节点与默认节点: 将默认配置中存在而用户配置中缺失的字段补充进去
YAML::Node MergeYamlNode(const YAML::Node& user_node,
                         const YAML::Node& default_node,
                         const std::string& path,
                         bool& modified) {
    YAML::Node result = user_node;             // 复制用户节点作为基础
    if (!default_node.IsMap()) return result;  // 只有字典(Map)类型才需要向下遍历合并

    for (const auto& default_field : default_node) {
        std::string field_key = default_field.first.as<std::string>();
        std::string full_path = path.empty() ? field_key : path + "." + field_key;

        if (!result[field_key]) {
            // 用户配置中不存在该字段，将默认配置中的整个节点补充进去
            result[field_key] = default_field.second;
            modified = true;
            std::cout << "[WARN] Added missing field: " << full_path << std::endl;
        } else if (default_field.second.IsMap()) {
            // 字段存在且为字典类型，递归合并其内部的子字段
            result[field_key] = MergeYamlNode(result[field_key], default_field.second, full_path, modified);
        }
    }
    return result;
}
}  // namespace

void CheckConfig(const std::filesystem::path& default_cfg_path, const std::filesystem::path& user_cfg_path) {
    if (!std::filesystem::exists(user_cfg_path)) {
        // 用户配置文件不存在，执行完全拷贝
        if (!std::filesystem::copy_file(default_cfg_path, user_cfg_path,
                                        std::filesystem::copy_options::overwrite_existing)) {
            std::cerr << "[ERROR] Failed to copy default config file: " << user_cfg_path << std::endl;
            std::exit(1);
        }
        std::cout << "[INFO] Created default config file: " << user_cfg_path << std::endl;
    } else {
        // 加载默认配置和用户修改过的配置进行对比合并
        YAML::Node default_config = YAML::LoadFile(default_cfg_path.string());
        YAML::Node user_config = YAML::LoadFile(user_cfg_path.string());

        // 检查配置版本，如果默认配置版本领先，则直接覆盖历史用户配置
        std::string default_version =
            default_config["config_version"] ? default_config["config_version"].as<std::string>() : "";
        std::string user_version = user_config["config_version"] ? user_config["config_version"].as<std::string>() : "";

        // 字符串比较版本号，假设版本格式为 "vXX"，直接比较字符串即可判断版本先后, 如果用户版本不存在则视为过旧需要覆盖
        if (default_version > user_version) {
            std::cout << "[INFO] Default config version (" << default_version << ") is newer than user config version ("
                      << (user_version.empty() ? "none" : user_version) << "). Overwriting user config." << std::endl;
            if (!std::filesystem::copy_file(default_cfg_path, user_cfg_path,
                                            std::filesystem::copy_options::overwrite_existing)) {
                std::cerr << "[ERROR] Failed to overwrite config file: " << user_cfg_path << std::endl;
                std::exit(1);
            }
            // 覆盖后重新加载最新的配置内容，以便于继续执行后续正常的一致性动作
            user_config = YAML::LoadFile(user_cfg_path.string());
        }

        bool modified = false;

        for (const auto& default_node : default_config) {
            std::string key = default_node.first.as<std::string>();

            if (!user_config[key]) {
                // 顶层字段缺失，补充整个节点
                user_config[key] = default_node.second;
                modified = true;
                std::cout << "[WARN] Added missing field: " << key << std::endl;
            } else if (default_node.second.IsMap()) {
                // 顶层字典类型字段存在，但内部二级字段可能缺失，遍历子字典处理
                for (const auto& sub_node : default_node.second) {
                    std::string sub_key = sub_node.first.as<std::string>();
                    if (!user_config[key][sub_key]) {
                        user_config[key][sub_key] = sub_node.second;
                        modified = true;
                        std::cout << "[WARN] Added missing field: " << key << "." << sub_key << std::endl;
                    }
                }
            } else if (default_node.second.IsSequence() && key == "joints") {
                // 特殊处理 joints 参数，joints 是一个数组，每个元素包含各个关节的具体配置
                for (size_t i = 0; i < default_node.second.size(); ++i) {
                    if (i >= user_config[key].size()) {
                        // 如果用户配置的关节数少于默认配置的关节数，将新增的关节补充到末尾
                        user_config[key].push_back(default_node.second[i]);
                        modified = true;
                        std::cout << "[WARN] Added missing joint at index: " << i << std::endl;
                    } else {
                        // 关节参数虽然存在但也可能缺失某些下级属性，调用合并函数进行属性补齐
                        user_config[key][i] = MergeYamlNode(user_config[key][i], default_node.second[i],
                                                            "joints[" + std::to_string(i) + "]", modified);
                    }
                }
            }
        }

        if (modified) {
            // 如果上述检查补齐了任何字段，将完整的用户配置覆盖写回原文件
            std::ofstream fout(user_cfg_path.string());
            fout << user_config;
            fout.close();
            std::cout << "[INFO] Updated config file with missing fields: " << user_cfg_path << std::endl;
        }
    }
}

void SyncHardwareConfig(const std::filesystem::path& robot_cfg_path) {
    const std::string external_cfg_path = std::string(std::getenv("HOME")) + "/.cache/dexe_data/config.yaml";
    if (!std::filesystem::exists(external_cfg_path)) {
        std::cout << "[INFO] External hardware config not found, skipping sync." << std::endl;
        return;
    }

    try {
        YAML::Node external_config = YAML::LoadFile(external_cfg_path);
        YAML::Node robot_config = YAML::LoadFile(robot_cfg_path.string());

        bool modified = false;
        for (auto const& it : external_config) {
            std::string key = it.first.as<std::string>();
            robot_config[key] = it.second;
            modified = true;
        }

        if (modified) {
            std::ofstream fout(robot_cfg_path.string());
            fout << robot_config;
            fout.close();
            std::cout << "[INFO] Synchronized all hardware fields from " << external_cfg_path << " to "
                      << robot_cfg_path << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to sync hardware config: " << e.what() << std::endl;
    }
}

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

    model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.8);
    left_arm_model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.8);
    right_arm_model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.8);

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

    if (version_cfg["enable_cartesian_impedance"]) {
        robot_property.enable_cartesian_impedance = version_cfg["enable_cartesian_impedance"].as<bool>();
    }
    if (config["allow_force_sensor"]) {
        robot_property.allow_force_sensor = config["allow_force_sensor"].as<bool>();
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

    auto read_joint_admittance_finite = [](const YAML::Node& node, const char* key, double& target) {
        if (!node || !node[key]) {
            return;
        }
        const double value = node[key].as<double>();
        if (!std::isfinite(value)) {
            LOG_WARN << "Invalid joint_admittance." << key << ": " << value << ", keep default " << target;
            return;
        }
        target = value;
    };
    auto read_joint_admittance_bounded = [](const YAML::Node& node, const char* key, double& target, double min_value,
                                            bool strict_min) {
        if (!node || !node[key]) {
            return;
        }
        const double value = node[key].as<double>();
        const bool valid_min = strict_min ? value > min_value : value >= min_value;
        if (!std::isfinite(value) || !valid_min) {
            LOG_WARN << "Invalid joint_admittance." << key << ": " << value << ", keep default " << target;
            return;
        }
        target = value;
    };
    auto read_joint_admittance_bool = [](const YAML::Node& node, const char* key, bool& target) {
        if (node && node[key]) {
            target = node[key].as<bool>();
        }
    };
    auto apply_joint_admittance_node = [&](const YAML::Node& node, JointAdmittanceJointConfig& target) {
        if (!node || !node.IsMap()) {
            return;
        }
        read_joint_admittance_bool(node, "enabled", target.enabled);
        read_joint_admittance_finite(node, "torque_gain", target.torque_gain);
        read_joint_admittance_bounded(node, "virtual_inertia", target.virtual_inertia, 0.0, true);
        read_joint_admittance_bounded(node, "natural_frequency_hz", target.natural_frequency_hz, 0.0, true);
        read_joint_admittance_bounded(node, "damping_ratio", target.damping_ratio, 0.0, true);
        read_joint_admittance_bounded(node, "offset_limit", target.offset_limit, 0.0, false);
        read_joint_admittance_bounded(node, "velocity_limit", target.velocity_limit, 0.0, false);
        read_joint_admittance_bounded(node, "acceleration_limit", target.acceleration_limit, 0.0, false);
    };

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

            if (joint_config["joint_admittance"] && joint_config["joint_admittance"].IsMap()) {
                apply_joint_admittance_node(joint_config["joint_admittance"],
                                            robot_property.joint_admittance.joints[joint_idx]);
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

    if (config["force_sensor_external_wrench"] && config["force_sensor_external_wrench"].IsMap()) {
        const auto external_wrench = config["force_sensor_external_wrench"];
        auto parse_non_negative = [&](const char* key, double& target) {
            if (!external_wrench[key]) {
                return;
            }
            const double value = external_wrench[key].as<double>();
            if (!std::isfinite(value) || value < 0.0) {
                LOG_WARN << "Invalid force_sensor_external_wrench." << key << ": " << value << ", keep default "
                         << target;
                return;
            }
            target = value;
        };
        auto parse_model_sign = [&](const char* key, int& target) {
            if (!external_wrench[key]) {
                return;
            }
            const int value = external_wrench[key].as<int>();
            if (value != -1 && value != 1) {
                LOG_WARN << "Invalid force_sensor_external_wrench." << key << ": " << value << ", keep default "
                         << target;
                return;
            }
            target = value;
        };
        parse_non_negative("force_deadband_n", params.external_wrench_force_deadband_n);
        parse_non_negative("force_saturation_n", params.external_wrench_force_saturation_n);
        parse_non_negative("torque_deadband_nm", params.external_wrench_torque_deadband_nm);
        parse_non_negative("torque_saturation_nm", params.external_wrench_torque_saturation_nm);
        parse_model_sign("model_sign", params.external_wrench_model_sign);
    }

    if (config["joint_admittance"] && config["joint_admittance"].IsMap()) {
        const auto admittance = config["joint_admittance"];
        read_joint_admittance_bool(admittance, "enabled", robot_property.joint_admittance.enabled);
        read_joint_admittance_finite(admittance, "torque_ext_direct_gain",
                                     robot_property.joint_admittance.torque_ext_direct_gain);

        if (admittance["default"]) {
            for (auto& joint_cfg : robot_property.joint_admittance.joints) {
                apply_joint_admittance_node(admittance["default"], joint_cfg);
            }
        }

        if (admittance["groups"] && admittance["groups"].IsMap()) {
            for (const auto& group : admittance["groups"]) {
                const auto group_node = group.second;
                if (!group_node["joints"] || !group_node["joints"].IsSequence()) {
                    LOG_WARN << "joint_admittance group missing joints list, skipping";
                    continue;
                }
                for (const auto& joint_name_node : group_node["joints"]) {
                    const std::string joint_name = joint_name_node.as<std::string>();
                    const auto it = std::find(params.joint_names.begin(), params.joint_names.end(), joint_name);
                    if (it == params.joint_names.end()) {
                        LOG_WARN << "joint_admittance unknown joint: " << joint_name;
                        continue;
                    }
                    const int joint_idx = std::distance(params.joint_names.begin(), it);
                    auto& joint_cfg = robot_property.joint_admittance.joints[joint_idx];
                    joint_cfg.enabled = true;
                    apply_joint_admittance_node(group_node, joint_cfg);
                }
            }
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

EERotationParseResult GetEERotationWorldFromPoseVector(const Eigen::VectorXd& ee_pose_world, int arm_index) {
    EERotationParseResult result;
    if (arm_index < 0) {
        result.reason = EERotationParseError::InvalidArmIndex;
        return result;
    }

    const int offset = arm_index * 7;
    result.required_size = offset + 7;
    if (ee_pose_world.size() < result.required_size) {
        result.reason = EERotationParseError::InsufficientSize;
        return result;
    }

    Eigen::Quaterniond q(ee_pose_world[offset + 3], ee_pose_world[offset + 4], ee_pose_world[offset + 5],
                         ee_pose_world[offset + 6]);
    if (!q.coeffs().allFinite()) {
        result.reason = EERotationParseError::NonFiniteQuaternion;
        return result;
    }
    if (q.norm() <= FloatZeroTol) {
        result.reason = EERotationParseError::NearZeroQuaternion;
        return result;
    }

    q.normalize();
    result.rotation = q.toRotationMatrix();
    result.reason = EERotationParseError::None;
    return result;
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

int GetTerminalBodyFrameId(const pinocchio::Model& model, const std::vector<std::string>& joint_names) {
    if (joint_names.empty()) {
        return -1;
    }

    const auto joint_id = model.getJointId(joint_names.back());
    if (joint_id == 0 || joint_id >= static_cast<pinocchio::JointIndex>(model.njoints)) {
        return -1;
    }

    int terminal_frame_id = -1;
    for (std::size_t frame_id = 0; frame_id < model.frames.size(); ++frame_id) {
        const auto& frame = model.frames[frame_id];
        if (frame.parentJoint == joint_id && frame.type == pinocchio::BODY) {
            terminal_frame_id = static_cast<int>(frame_id);
        }
    }
    return terminal_frame_id;
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
        case RequestType::EnableChassisShaper:
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
    Vector12d sensor_wrench;
    sensor_wrench.head<6>() = wrench_primary;
    sensor_wrench.tail<6>() = wrench_secondary;
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
