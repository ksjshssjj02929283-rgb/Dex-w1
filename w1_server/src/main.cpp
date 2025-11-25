#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

#include "chassis_node.h"
#include "w1_server.h"

static void setup_realtime()
{
    // 锁内存，避免分页抖动
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
        std::cerr << "mlockall failed: " << std::strerror(errno) << std::endl;
    }
    // 设置SCHED_FIFO实时调度
    sched_param sp{};
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
    {
        std::cerr << "sched_setscheduler failed: " << std::strerror(errno) << std::endl;
        LOG_CRIT << "sched_setscheduler failed, please run setcap!";
        assert(false && "sched_setscheduler failed, please run setcap!");
    }
}

int main(int argc, char* argv[])
{
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";

    rclcpp::init(argc, argv);

    // 设置实时优先级
    setup_realtime();
    // 启动节点
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>(robot_cfg_path, false);
    auto node_chassis = std::make_shared<chassis_node::ChassisNode>();
    executor.add_node(node_server);
    executor.add_node(node_chassis);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
