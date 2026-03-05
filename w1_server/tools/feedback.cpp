#include <pwd.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <thread>

#include "NanoLog.hpp"
#include "shared_memory_def.h"

volatile std::sig_atomic_t g_stop_flag = 0;
const int ndof_robot = 20;  // 机器人实际关节数，Arm/A1 模式下只使用前 7/14 个

void signal_handler(int signal) {
    if (signal == SIGINT) {
        g_stop_flag = 1;
    }
}

static void setup_realtime() {
    // 锁内存，避免分页抖动
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "mlockall failed: " << std::strerror(errno) << std::endl;
    }
    // 设置SCHED_FIFO实时调度
    sched_param sp{};
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::cerr << "sched_setscheduler failed: " << std::strerror(errno) << std::endl;
    }
}

// Feedbacker: 写入RobotFeedback，读取RobotControl
int main(int argc, char* argv[]) {
    setup_realtime();

    // 检查是否需要记录日志和机器人模式
    bool record = false;
    int ndof = ndof_robot;  // 默认 20 关节 (W1 模式)
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--record") {
            record = true;
        } else if (arg == "--a1" || arg == "a1") {
            ndof = 14;  // A1 模式 14 关节
        } else if (arg == "--arm" || arg == "arm") {
            ndof = 7;  // Arm 模式 7 关节
        }
    }

    // 如果需要记录日志，初始化日志系统
    if (record) {
        const char* home_dir = std::getenv("HOME");
        std::filesystem::path log_dir = std::filesystem::path(home_dir) / "log";
        std::filesystem::create_directories(log_dir);
        nanolog::initialize(nanolog::GuaranteedLogger(), (log_dir / "").string(), "feedbacker", 10);
        nanolog::set_log_level(nanolog::LogLevel::INFO);
    }

    std::signal(SIGINT, signal_handler);
    auto& fb_shm = FeedbackSharedMemory::GetInstance();
    auto& ctrl_shm = ControlSharedMemory::GetInstance();

    while (!g_stop_flag) {
        // 读取控制指令
        RobotFeedback fb;
        RobotControl ctrl;
        ctrl.position[0] = 0.5;
        ctrl.position[1] = -1.0;
        ctrl.position[2] = 0.5;
        if (ctrl_shm.PeekLatest(&ctrl)) {
            // 如果需要记录日志，只记录速度数据
            if (record) {
                std::ostringstream ss;
                ss << "vel: ";
                for (int i = 0; i < ndof; i++) {
                    ss << ctrl.velocity[i];
                    if (i < ndof - 1) ss << ", ";
                }
                LOG_INFO << ss.str();
            }
        }

        // 构造反馈数据并写入
        fb.robot_status_word = 2;  // robot ready
        for (int i = 0; i < ndof; i++) {
            fb.position[i] = ctrl.position[i];
            fb.velocity[i] = ctrl.velocity[i];
            fb.torque[i] = ctrl.torque_feedforward[i];
        }
        if (fb_shm.AppendNewElement(fb)) {
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
