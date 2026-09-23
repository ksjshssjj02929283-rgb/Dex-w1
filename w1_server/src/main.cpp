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
    if (!std::filesystem::exists(robot_cfg_path)) {
        std::cerr << "[ERROR] Runtime config file does not exist: " << robot_cfg_path << std::endl;
        return 1;
    }
    std::cout << "Using runtime config file: " << robot_cfg_path << std::endl;

    // 直接使用 install 中的配置文件；不再复制到 HOME 下的运行时目录。
    robot_server_lib::utils::SyncHardwareConfig(robot_cfg_path);
    std::cout << "[INFO] Mode: 自动检测模式（根据实际机器人关节配置）" << std::endl;
    // 初始化RCLCPP
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>(robot_cfg_path, false);
    std::shared_ptr<chassis_node::ChassisNode> node_chassis;
    // RobotServer 与 ChassisNode 必须使用同一份 install 配置；
    // 否则一个节点可能判定为全向、另一个节点仍按差速发布反馈。
    // 两个节点共用同一条底盘目标通道：chassis_node 写、RobotServer 的 1 kHz 线程读。
    node_chassis = std::make_shared<chassis_node::ChassisNode>(robot_cfg_path, node_server->chassis_target_buffer());
    executor.add_node(node_chassis);
    executor.add_node(node_server);
    SetupRealtimeThread(70);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
