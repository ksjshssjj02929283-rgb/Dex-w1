#include <pwd.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

#include "shared_memory_def.h"

volatile std::sig_atomic_t g_stop_flag = 0;
static double prev_position[20] = {0};

std::string w1_joint_name_[20] = {"ANKLE",    "KNEE",     "BUTTOCK",  "WAIST",    "NECK1",    "NECK2",   "LEFT_J1",
                                  "LEFT_J2",  "LEFT_J3",  "LEFT_J4",  "LEFT_J5",  "LEFT_J6",  "LEFT_J7", "RIGHT_J1",
                                  "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"};

std::string a1_joint_name_[16] = {"NECK1",    "NECK2",    "LEFT_J1",  "LEFT_J2",  "LEFT_J3",  "LEFT_J4",
                                  "LEFT_J5",  "LEFT_J6",  "LEFT_J7",  "RIGHT_J1", "RIGHT_J2", "RIGHT_J3",
                                  "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"};

std::string dual_arm_joint_name_[14] = {"LEFT_J1",  "LEFT_J2",  "LEFT_J3",  "LEFT_J4",  "LEFT_J5",
                                        "LEFT_J6",  "LEFT_J7",  "RIGHT_J1", "RIGHT_J2", "RIGHT_J3",
                                        "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"};

std::string single_arm_joint_name_[7] = {"LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7"};

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

    int ndof = sizeof(w1_joint_name_) / sizeof(w1_joint_name_[0]);
    std::string* joint_names = w1_joint_name_;
    std::string mode = "w1";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--a1" || arg == "a1") {
            ndof = sizeof(a1_joint_name_) / sizeof(a1_joint_name_[0]);
            joint_names = a1_joint_name_;
            mode = "a1";
        } else if (arg == "--single" || arg == "single" || arg == "--arm" || arg == "arm") {
            ndof = sizeof(single_arm_joint_name_) / sizeof(single_arm_joint_name_[0]);
            joint_names = single_arm_joint_name_;
            mode = "single";
        } else if (arg == "--dual" || arg == "dual") {
            ndof = sizeof(dual_arm_joint_name_) / sizeof(dual_arm_joint_name_[0]);
            joint_names = dual_arm_joint_name_;
            mode = "dual";
        }
    }

    std::cout << "Starting feedback in " << mode << " mode with " << ndof << " joints" << std::endl;

    std::signal(SIGINT, signal_handler);
    auto& fb_shm = FeedbackSharedMemory::GetInstance();
    auto& ctrl_shm = ControlSharedMemory::GetInstance();

    int loop_counter = 0;
    while (!g_stop_flag) {
        loop_counter++;
        RobotFeedback fb{};
        RobotControl ctrl{};
        ctrl.joint_control[0].joint_position = 0.5;
        ctrl.joint_control[1].joint_position = -1.0;
        ctrl.joint_control[2].joint_position = 0.5;
        ctrl_shm.PeekLatest(&ctrl);

        fb.robot_status_word = 2;
        for (int i = 0; i < ndof; i++) {
            int reverse_idx = ndof - 1 - i;
            std::strncpy(fb.joint_feedback[i].joint_name, joint_names[reverse_idx].c_str(),
                         sizeof(fb.joint_feedback[i].joint_name) - 1);
            fb.joint_feedback[i].joint_name[sizeof(fb.joint_feedback[i].joint_name) - 1] = '\0';
            fb.joint_feedback[i].joint_status_word = 8;
            fb.joint_feedback[i].joint_position = ctrl.joint_control[reverse_idx].joint_position;
            fb.joint_feedback[i].joint_velocity =
                (ctrl.joint_control[reverse_idx].joint_position - prev_position[reverse_idx]) * 1000.0;
            prev_position[reverse_idx] = ctrl.joint_control[reverse_idx].joint_position;
            fb.joint_feedback[i].joint_torque = ctrl.joint_control[reverse_idx].joint_torque_feedforward;
        }
        if (loop_counter % 100 == 0) {
            std::cout << "--- " << mode << " Mode Joint Names & Positions ---" << std::endl;
            for (int i = 0; i < ndof; i++) {
                std::cout << "  " << fb.joint_feedback[i].joint_name << ": " << fb.joint_feedback[i].joint_position
                          << std::endl;
            }
        }
        fb.imu_data.orientation.w = 1.0f;
        fb.imu_data.angular_velocity = {0.0f, 0.0f, 0.0f};
        fb.imu_data.linear_acceleration = {0.0f, 0.0f, 9.7887f};
        fb.imu_data.mag = {0.0f, 0.0f, 0.0f};
        fb.imu_data.eul = {0.0f, 0.0f, 0.0f};

        fb_shm.AppendNewElement(fb);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
