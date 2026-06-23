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
const int ndof_body = 18;
const int ndof_legs = 12;
const int ndof_robot = ndof_body + ndof_legs;

void signal_handler(int signal) {
    if (signal == SIGINT) {
        g_stop_flag = 1;
    }
}

static void setup_realtime() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "mlockall failed: " << std::strerror(errno) << std::endl;
    }
    sched_param sp{};
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::cerr << "sched_setscheduler failed: " << std::strerror(errno) << std::endl;
    }
}

int main(int argc, char* argv[]) {
    setup_realtime();

    bool record = false;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--record") {
            record = true;
        }
    }

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

    RobotFeedback fb{};
    RobotControl ctrl{};

    while (!g_stop_flag) {
        if (ctrl_shm.PeekLatest(&ctrl)) {
            if (record) {
                std::ostringstream ss;
                ss << "ctrl position: ";
                for (int i = 0; i < ndof_robot; i++) {
                    ss << ctrl.joint_control[i].joint_position;
                    if (i < ndof_robot - 1) ss << ", ";
                }
                LOG_INFO << ss.str();
            }
        }

        fb.robot_status_word = 2;

        for (int i = 0; i < ndof_body; i++) {
            fb.joint_feedback[i].joint_position = ctrl.joint_control[i].joint_position;
            fb.joint_feedback[i].joint_velocity = ctrl.joint_control[i].joint_velocity;
            fb.joint_feedback[i].joint_torque = ctrl.joint_control[i].joint_torque_feedforward;
        }

        for (int i = 0; i < ndof_legs; i++) {
            int idx = i + ndof_body;
            fb.joint_feedback[idx].joint_position = ctrl.joint_control[idx].joint_position;
            fb.joint_feedback[idx].joint_velocity = ctrl.joint_control[idx].joint_velocity;
            fb.joint_feedback[idx].joint_torque = ctrl.joint_control[idx].joint_torque;
            fb.joint_feedback[idx].joint_status_word = 2;
        }
        fb_shm.AppendNewElement(fb);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
