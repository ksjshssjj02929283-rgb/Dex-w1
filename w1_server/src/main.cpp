#include <pthread.h>
#include <sched.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "chassis_node.h"
#include "w1_server.h"

static void SetupRealtimeThread(int priority) {
    pthread_t this_thread = pthread_self();
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(this_thread, SCHED_FIFO, &param) != 0) {
        std::cerr << "pthread_setschedparam failed: " << std::strerror(errno) << std::endl;
    }
}

int main(int argc, char* argv[]) {
    // 初始化配置文件
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    std::string cfg_file = "robot_config.yaml";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / cfg_file;
    std::cout << "Using config file: " << robot_cfg_path << std::endl;

    std::filesystem::path user_cfg_path;
    user_cfg_path = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data" / cfg_file;
    if (!std::filesystem::exists(user_cfg_path.parent_path())) {
        std::filesystem::create_directories(user_cfg_path.parent_path());
        std::cout << "[INFO] Created data directory: " << user_cfg_path.parent_path() << std::endl;
    }
    robot_server_lib::utils::CheckConfig(robot_cfg_path, user_cfg_path);

    robot_server_lib::utils::SyncHardwareConfig(robot_cfg_path);
    if (std::filesystem::exists(user_cfg_path)) {
        robot_server_lib::utils::SyncHardwareConfig(user_cfg_path);
    }
    std::cout << "[INFO] Mode: 自动检测模式（根据实际机器人关节配置）" << std::endl;
    // 初始化RCLCPP
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>(user_cfg_path, false);
    std::shared_ptr<chassis_node::ChassisNode> node_chassis;
    // 模式在运行时由反馈自动识别，启动阶段尚不可知，故底盘节点无条件创建；
    // 无底盘的机型下它读不到 chassis_feedback，会自行保持静默。
    node_chassis = std::make_shared<chassis_node::ChassisNode>();
    executor.add_node(node_chassis);
    executor.add_node(node_server);
    SetupRealtimeThread(70);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
