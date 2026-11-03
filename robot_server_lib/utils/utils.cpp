#include "utils.h"

#include <unistd.h>

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
        } else if (field_key == "joints" && default_field.second.IsSequence()) {
            // 版本配置下的 joints 列表也需补齐单关节属性, 保留已有参数。
            for (size_t i = 0; i < default_field.second.size(); ++i) {
                if (i >= result[field_key].size()) {
                    result[field_key].push_back(default_field.second[i]);
                    modified = true;
                    std::cout << "[WARN] Added missing joint: " << full_path << "[" << i << "]" << std::endl;
                } else {
                    result[field_key][i] = MergeYamlNode(result[field_key][i], default_field.second[i],
                                                         full_path + "[" + std::to_string(i) + "]", modified);
                }
            }
        }
    }
    // Keep the four tuning arrays immediately after the identified parameters.
    if (result["dyn_base_params_secondary"]) {
        const std::vector<std::string> ratio_keys{"fv_ratio_primary", "fv_ratio_secondary", "fc_ratio_primary",
                                                  "fc_ratio_secondary"};
        YAML::Node ordered(YAML::NodeType::Map);
        std::vector<std::string> before, after;
        for (const auto& field : result) {
            const auto key = field.first.as<std::string>();
            before.push_back(key);
            if (std::find(ratio_keys.begin(), ratio_keys.end(), key) != ratio_keys.end()) continue;
            ordered[key] = field.second;
            if (key == "dyn_base_params_secondary") {
                for (const auto& ratio_key : ratio_keys) {
                    if (result[ratio_key]) {
                        ordered[ratio_key] = result[ratio_key];
                        ordered[ratio_key].SetStyle(YAML::EmitterStyle::Flow);
                    }
                }
            }
        }
        for (const auto& field : ordered) after.push_back(field.first.as<std::string>());
        if (before != after) modified = true;
        return ordered;
    }
    return result;
}

// 原子写入 YAML 配置: 先写临时文件(PID 命名避免多进程冲突), 再 rename 原子替换
// 读者要么看到完整旧文件, 要么看到完整新文件, 不会读到半写状态
void AtomicWriteYaml(const std::filesystem::path& target_path, const YAML::Node& config) {
    std::filesystem::path tmp_path = target_path.string() + ".tmp." + std::to_string(::getpid());
    std::ofstream fout(tmp_path.string());
    if (!fout.is_open()) {
        std::cerr << "[ERROR] Failed to open temp file for writing: " << tmp_path << std::endl;
        return;
    }
    fout << config;
    fout.close();
    if (!fout) {
        std::cerr << "[ERROR] Failed to write config to temp file: " << tmp_path << std::endl;
        std::filesystem::remove(tmp_path);
        return;
    }
    std::filesystem::rename(tmp_path, target_path);
}
}  // namespace

void CheckConfig(const std::filesystem::path& default_cfg_path, const std::filesystem::path& user_cfg_path) {
    if (!std::filesystem::exists(user_cfg_path)) {
        // 用户配置文件不存在，用默认配置原子写入创建
        YAML::Node fresh_config = YAML::LoadFile(default_cfg_path.string());
        AtomicWriteYaml(user_cfg_path, fresh_config);
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
                      << (user_version.empty() ? "none" : user_version) << "). Replacing user config." << std::endl;
            // 先删除旧文件, 再用默认配置原子写入, 避免非原子 copy_file 导致并发撕裂
            std::filesystem::remove(user_cfg_path);
            YAML::Node fresh_config = YAML::LoadFile(default_cfg_path.string());
            AtomicWriteYaml(user_cfg_path, fresh_config);
            user_config = fresh_config;
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
                // 递归补齐版本配置及其中的关节属性。
                user_config[key] = MergeYamlNode(user_config[key], default_node.second, key, modified);
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
            // 原子写入: 先写临时文件再 rename, 避免多进程并发写导致文件撕裂
            AtomicWriteYaml(user_cfg_path, user_config);
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
            AtomicWriteYaml(robot_cfg_path, robot_config);
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

    {
        const auto tool_dyn_node = config["tool_dynamics"];
        auto parse_tool = [&tool_dyn_node, &params](const std::string& hand_type, bool enabled,
                                                   dynamics::tool_compensation::ToolDynamicsConfig& tool_cfg,
                                                   const char* arm_name, const char* side) {
            if (!tool_dyn_node || !enabled || hand_type.empty() || hand_type == "null") {
                return;
            }
            auto is_valid_tool_node = [](const YAML::Node& n) {
                return n && ((n["use_urdf"] && n["use_urdf"].as<bool>()) ||
                             (n["com_m"] && n["com_m"].IsSequence() && n["com_m"].size() == 3 && n["mass_kg"]));
            };
            const auto tool_node = tool_dyn_node[hand_type];
            const auto default_node = tool_dyn_node["Default"];
            YAML::Node selected_node;
            std::string selected_type;
            if (is_valid_tool_node(tool_node)) {
                selected_node = tool_node;
                selected_type = hand_type;
            } else if (is_valid_tool_node(default_node)) {
                selected_node = default_node;
                selected_type = "Default";
                std::cout << "[INFO] Tool load compensation: tool_dynamics[" << hand_type
                          << "] not matched, falling back to Default (" << arm_name << ")\n";
                LOG_INFO << "Tool load compensation: tool_dynamics[" << hand_type
                         << "] not matched, falling back to Default (" << arm_name << ")";
            } else {
                std::cout << "[WARN] Tool load compensation: tool_dynamics[" << hand_type
                          << "] missing or invalid and no valid Default, disabled (" << arm_name << ")\n";
                LOG_WARN << "Tool load compensation: tool_dynamics[" << hand_type
                         << "] missing or invalid and no valid Default, disabled (" << arm_name << ")";
                return;
            }
            if (selected_node["use_urdf"] && selected_node["use_urdf"].as<bool>()) {
                const auto tool_urdf_path = dynamics::tool_compensation::ResolveToolUrdfPath(
                    params.robot_version, hand_type, side);
                tool_cfg.inertia = dynamics::tool_compensation::LoadToolInertia(tool_urdf_path);
                tool_cfg.enabled = true;
                LOG_INFO << "Tool load URDF (" << arm_name << "): " << tool_urdf_path.string();
            } else {
                const double mass_kg = selected_node["mass_kg"].as<double>();
                Vector3d com_m;
                for (int i = 0; i < 3; ++i) {
                    com_m[i] = selected_node["com_m"][i].as<double>();
                }
                if (!std::isfinite(mass_kg) || mass_kg <= FloatZeroTol || !com_m.allFinite()) {
                    LOG_WARN << "Tool load compensation disabled (" << arm_name
                             << "): invalid mass_kg/com_m for " << selected_type;
                    return;
                }
                tool_cfg.inertia = pinocchio::Inertia(mass_kg, com_m, Matrix3d::Zero());
                tool_cfg.enabled = true;
            }
            std::cout << "[INFO] Tool load compensation enabled (" << arm_name << ", type=" << selected_type
                      << "): mass_kg=" << tool_cfg.inertia.mass()
                      << ", com_m=" << tool_cfg.inertia.lever().transpose() << "\n";
            LOG_INFO << "Tool load compensation enabled (" << arm_name << ", type=" << selected_type
                     << "): mass_kg=" << tool_cfg.inertia.mass()
                     << ", com_m=" << tool_cfg.inertia.lever().transpose();
        };
        const bool primary_enabled =
            tool_dyn_node && tool_dyn_node["tool_primary_enabled"] && tool_dyn_node["tool_primary_enabled"].as<bool>();
        const bool secondary_enabled = tool_dyn_node && tool_dyn_node["tool_secondary_enabled"] &&
                                       tool_dyn_node["tool_secondary_enabled"].as<bool>();
        parse_tool(params.left_hand_type, primary_enabled, params.tool_dynamic_primary, "primary", "left");
        parse_tool(params.right_hand_type, secondary_enabled, params.tool_dynamic_secondary, "secondary", "right");
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

    auto parse_friction_ratio = [&](const char* key, std::array<double, 7>& dst) {
        const auto node = version_cfg[key];
        if (!node) return;  // Defaults also support callers that do not run CheckConfig.
        if (!node.IsSequence() || node.size() != dst.size()) {
            throw std::runtime_error(std::string(key) + " must contain exactly 7 ratios");
        }
        for (size_t i = 0; i < dst.size(); ++i) {
            const double value = node[i].as<double>();
            if (!std::isfinite(value) || value < 0.0) {
                throw std::runtime_error(std::string(key) + " ratios must be finite and nonnegative");
            }
            dst[i] = value;
        }
    };
    parse_friction_ratio("fv_ratio_primary", params.fv_ratio_primary);
    parse_friction_ratio("fv_ratio_secondary", params.fv_ratio_secondary);
    parse_friction_ratio("fc_ratio_primary", params.fc_ratio_primary);
    parse_friction_ratio("fc_ratio_secondary", params.fc_ratio_secondary);
    std::array<double, 7> drag_damping_d2_primary{};
    std::array<double, 7> drag_damping_d2_secondary{};
    auto parse_drag_damping = [&](const char* key, std::array<double, 7>& coefficients, bool positive = false) {
        const auto node = version_cfg[key];
        if (!node) return;
        if (!node.IsSequence() || node.size() != coefficients.size()) {
            throw std::runtime_error(std::string(key) + " must contain exactly 7 coefficients");
        }
        for (size_t joint = 0; joint < coefficients.size(); ++joint) {
            const double value = node[joint].as<double>();
            if (!std::isfinite(value) || (positive ? value <= 0.0 : value < 0.0)) {
                throw std::runtime_error(std::string(key) + " coefficients must be finite and " +
                                         (positive ? "positive" : "nonnegative"));
            }
            coefficients[joint] = value;
        }
    };
    parse_drag_damping("drag_damping_d2_primary", drag_damping_d2_primary);
    parse_drag_damping("drag_damping_d2_secondary", drag_damping_d2_secondary);
    std::array<double, 7> drag_dq_lowspeed_primary{}, drag_dq_lowspeed_secondary{};
    drag_dq_lowspeed_primary.fill(1.0);
    drag_dq_lowspeed_secondary.fill(1.0);
    parse_drag_damping("drag_dq_lowspeed_primary", drag_dq_lowspeed_primary, true);
    parse_drag_damping("drag_dq_lowspeed_secondary", drag_dq_lowspeed_secondary, true);
    std::array<double, 7> drag_f_dead_primary{}, drag_f_dead_secondary{};
    parse_drag_damping("f_dead_primary", drag_f_dead_primary);
    parse_drag_damping("f_dead_secondary", drag_f_dead_secondary);
    if (version_cfg["v_dead"]) {
        params.drag_v_dead = version_cfg["v_dead"].as<double>();
        if (!std::isfinite(params.drag_v_dead) || params.drag_v_dead < 0.0) {
            throw std::runtime_error("v_dead must be finite and nonnegative");
        }
    }
    if (version_cfg["omega"]) {
        params.drag_omega = version_cfg["omega"].as<double>();
        if (!std::isfinite(params.drag_omega) || params.drag_omega < 0.0) {
            throw std::runtime_error("omega must be finite and nonnegative");
        }
    }
    if (version_cfg["friction_model"]) {
        params.friction_model = version_cfg["friction_model"].as<std::string>();
    }
    if (params.friction_model != "classic" && params.friction_model != "smooth" &&
        params.friction_model != "stribeck") {
        throw std::runtime_error("friction_model must be classic, smooth or stribeck");
    }
    auto parse_friction_value = [&](const char* key, double& value) {
        if (version_cfg[key]) value = version_cfg[key].as<double>();
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::runtime_error(std::string(key) + " must be finite and positive");
        }
    };
    parse_friction_value("friction_velocity_scale", params.friction_velocity_scale);
    parse_friction_value("stribeck_velocity", params.stribeck_velocity);
    parse_friction_value("stribeck_alpha", params.stribeck_alpha);

    robot_model.model_ptr = std::make_shared<pinocchio::Model>(model);
    robot_model.primary_arm_model_ptr = std::make_shared<pinocchio::Model>(left_arm_model);
    robot_model.secondary_arm_model_ptr = std::make_shared<pinocchio::Model>(right_arm_model);

    int num_joints = model.nq;
    params.joint_velocity_limit = Eigen::VectorXd::Zero(num_joints);
    params.joint_torque_limit = Eigen::VectorXd::Zero(num_joints);
    params.joint_limit_margin_deg = Eigen::VectorXd::Zero(num_joints);

    RobotProperty robot_property;

    if (version_cfg["allow_torque_control"]) {
        robot_property.allow_torque_control = version_cfg["allow_torque_control"].as<bool>();
    }

    if (version_cfg["enable_cartesian_impedance"]) {
        robot_property.enable_cartesian_impedance = version_cfg["enable_cartesian_impedance"].as<bool>();
    }

    // 关节导纳参数解析辅助: 非法值回退默认并告警, 保证控制器参数始终有效
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
        if (!target.enabled) {
            return;  // 关闭的关节跳过具体参数解析
        }
        read_joint_admittance_bounded(node, "natural_frequency_hz", target.natural_frequency_hz, 0.0, true);
        read_joint_admittance_bounded(node, "damping_ratio", target.damping_ratio, 0.0, true);
        read_joint_admittance_bounded(node, "offset_limit", target.offset_limit, 0.0, false);
        read_joint_admittance_bounded(node, "velocity_limit", target.velocity_limit, 0.0, false);
        read_joint_admittance_bounded(node, "acceleration_limit", target.acceleration_limit, 0.0, false);
    };

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

            const auto primary_joint =
                std::find(params.joints_primary_names.begin(), params.joints_primary_names.end(), joint_name);
            const auto secondary_joint =
                std::find(params.joints_secondary_names.begin(), params.joints_secondary_names.end(), joint_name);
            if (primary_joint != params.joints_primary_names.end()) {
                const auto arm_joint = std::distance(params.joints_primary_names.begin(), primary_joint);
                params.drag_damping_d2[joint_idx] = drag_damping_d2_primary.at(arm_joint);
                params.drag_dq_lowspeed[joint_idx] = drag_dq_lowspeed_primary.at(arm_joint);
                params.drag_f_dead[joint_idx] = drag_f_dead_primary.at(arm_joint);
            } else if (secondary_joint != params.joints_secondary_names.end()) {
                const auto arm_joint = std::distance(params.joints_secondary_names.begin(), secondary_joint);
                params.drag_damping_d2[joint_idx] = drag_damping_d2_secondary.at(arm_joint);
                params.drag_dq_lowspeed[joint_idx] = drag_dq_lowspeed_secondary.at(arm_joint);
                params.drag_f_dead[joint_idx] = drag_f_dead_secondary.at(arm_joint);
            }

            const bool is_arm_joint =
                std::find(params.joints_primary_names.begin(), params.joints_primary_names.end(), joint_name) !=
                    params.joints_primary_names.end() ||
                std::find(params.joints_secondary_names.begin(), params.joints_secondary_names.end(), joint_name) !=
                    params.joints_secondary_names.end();
            // 仅手臂支持配置外力矩死区; 缺失或非法值保留原来的 0.5 N·m。
            if (joint_config["torque_ext_deadzone"]) {
                if (!is_arm_joint) {
                    LOG_WARN << "torque_ext_deadzone ignored for non-arm joint: " << joint_name;
                } else {
                    try {
                        const double value = joint_config["torque_ext_deadzone"].as<double>();
                        if (std::isfinite(value) && value >= 0.0) {
                            params.joint_torque_ext_deadzone[joint_idx] = value;
                        } else {
                            LOG_WARN << "Invalid torque_ext_deadzone for joint " << joint_name << ": " << value
                                     << ", keep default " << TorqueExtDeadzone;
                        }
                    } catch (const YAML::Exception& e) {
                        LOG_WARN << "Invalid torque_ext_deadzone for joint " << joint_name << ": " << e.what()
                                 << ", keep default " << TorqueExtDeadzone;
                    }
                }
            }

            if (joint_config["velocity_limit"]) {
                params.joint_velocity_limit[joint_idx] = joint_config["velocity_limit"].as<double>();
            }

            if (joint_config["torque_limit"]) {
                params.joint_torque_limit[joint_idx] = joint_config["torque_limit"].as<double>();
            }

            if (joint_config["position_limit_margin_deg"]) {
                params.joint_limit_margin_deg[joint_idx] = joint_config["position_limit_margin_deg"].as<double>();
            }

            if (joint_config["joint_pd_control"]) {
                if (joint_config["joint_pd_control"]["stiffness"]) {
                    robot_property.joint_stiffness[joint_idx] =
                        joint_config["joint_pd_control"]["stiffness"].as<double>();
                }
                if (joint_config["joint_pd_control"]["damping"]) {
                    robot_property.joint_damping[joint_idx] = joint_config["joint_pd_control"]["damping"].as<double>();
                }
                if (joint_config["joint_pd_control"]["integral_ratio"]) {
                    double integral_ratio = joint_config["joint_pd_control"]["integral_ratio"].as<double>();
                    if (std::isfinite(integral_ratio) && integral_ratio >= 0.0) {
                        robot_property.joint_integral_ratio[joint_idx] = integral_ratio;
                    } else {
                        LOG_WARN << "Invalid integral_ratio for joint " << joint_name << ": " << integral_ratio
                                 << ", keep default " << robot_property.joint_integral_ratio[joint_idx];
                    }
                }
            }

            // 解析单关节导纳参数 (per-joint, 在 joints 列表中配置)
            if (joint_config["joint_admittance"] && joint_config["joint_admittance"].IsMap()) {
                apply_joint_admittance_node(joint_config["joint_admittance"],
                                            robot_property.joint_admittance.joints[joint_idx]);
                // 保险: 非手臂关节强制禁用导纳, 防止误配置
                if (!is_arm_joint && robot_property.joint_admittance.joints[joint_idx].enabled) {
                    LOG_WARN << "joint_admittance disabled for non-arm joint: " << joint_name;
                    robot_property.joint_admittance.joints[joint_idx].enabled = false;
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
