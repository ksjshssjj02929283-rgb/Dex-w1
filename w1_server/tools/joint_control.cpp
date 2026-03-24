#include <pwd.h>
#include <readline/history.h>
#include <readline/readline.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/select.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "NanoLog.hpp"
#include "shared_memory_def.h"
#include "sri_force_sensor.h"
#include "typedef.h"
#include "w1_utils.h"

volatile std::sig_atomic_t g_stop_flag = 0;
static bool g_got_input = false;
static std::string g_input_line;

void rl_get_input(char* prompt) {
    // 自定义输入函数，定期检查停止标志
    while (!g_got_input && !g_stop_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void input_callback(char* line) {
    if (line) {
        g_input_line = line;
        free(line);
    } else {
        g_input_line = "";
    }
    g_got_input = true;
}
static double target_position = 0.0;      // 目标位置
static double target_torque = 0.0;        // 目标力矩
static std::string control_mode = "csp";  // 控制模式: csp 或 cst

// 当前反馈数据（从控制线程更新）
static double current_position = 0.0;
static double current_velocity = 0.0;
static double current_torque = 0.0;
static uint16_t current_status_word = 0;

// 力传感器数据
static robot_server_lib::Vector6d sensor_force = robot_server_lib::Vector6d::Zero();

// 记录开关
static bool enable_record = false;

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

void print_help() {
    std::cout << "\n=== 单关节控制命令 ===\n"
              << "pos <value>   - 设置目标位置 (角度)\n"
              << "torque <value> - 设置目标力矩 (Nm)\n"
              << "mode <csp|cst> - 设置控制模式\n"
              << "record on|off  - 开启/关闭数据记录\n"
              << "status        - 查看当前状态\n"
              << "help          - 显示帮助\n"
              << "quit          - 退出程序\n"
              << "\n"
              << "===================\n\n";
}

void process_command(const std::string& line) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "pos" || cmd == "position") {
        double val;
        if (iss >> val) {
            target_position = val * M_PI / 180.0;  // 度转弧度
            std::cout << "设置目标位置: " << val << " deg\n";
        } else {
            std::cout << "用法: pos <value>\n";
        }
    } else if (cmd == "torque" || cmd == "t") {
        double val;
        if (iss >> val) {
            target_torque = val;
            std::cout << "设置目标力矩: " << val << " Nm\n";
        } else {
            std::cout << "用法: torque <value>\n";
        }
    } else if (cmd == "mode" || cmd == "m") {
        std::string mode;
        if (iss >> mode) {
            if (mode == "csp" || mode == "cst") {
                control_mode = mode;
                std::cout << "设置控制模式: " << mode << "\n";
            } else {
                std::cout << "无效模式，请使用: csp 或 cst\n";
            }
        } else {
            std::cout << "用法: mode <csp|cst>\n";
        }
    } else if (cmd == "status" || cmd == "s") {
        std::cout << "\n============= 当前状态 =============\n"
                  << "【指令】\n"
                  << "  目标位置: " << target_position * 180.0 / M_PI << " deg\n"
                  << "  目标力矩: " << target_torque << " Nm\n"
                  << "  控制模式: " << control_mode << "\n"
                  << "【反馈】\n"
                  << "  当前位置: " << current_position * 180.0 / M_PI << " deg\n"
                  << "  当前速度: " << current_velocity * 180.0 / M_PI << " deg/s\n"
                  << "  当前力矩: " << current_torque << " Nm\n"
                  << "  状态字: " << current_status_word << " ("
                  << ((current_status_word == STATUSWORD_MOTOR_READY)        ? "READY"
                      : (current_status_word == STATUSWORD_MOTOR_ENABLED)    ? "ENABLED"
                      : (current_status_word == STATUSWORD_MOTOR_FAULT)      ? "FAULT"
                      : (current_status_word == STATUSWORD_MOTOR_QUICK_STOP) ? "QUICK_STOP"
                                                                             : "NOT_READY")
                  << ")\n"
                  << "====================================\n\n";
    } else if (cmd == "help" || cmd == "h" || cmd == "?") {
        print_help();
    } else if (cmd == "record" || cmd == "r") {
        std::string onoff;
        if (iss >> onoff) {
            if (onoff == "on" || onoff == "1" || onoff == "true") {
                enable_record = true;
                std::cout << "开启数据记录\n";
            } else if (onoff == "off" || onoff == "0" || onoff == "false") {
                enable_record = false;
                std::cout << "关闭数据记录\n";
            } else {
                std::cout << "用法: record on|off\n";
            }
        } else {
            std::cout << "当前记录状态: " << (enable_record ? "开启" : "关闭") << "\n";
        }
    } else if (cmd == "quit" || cmd == "q" || cmd == "exit") {
        g_stop_flag = 1;
    } else if (cmd.empty()) {
        // 忽略空行
    } else {
        std::cout << "未知命令: " << cmd << ", 输入 help 查看帮助\n";
    }
}

int main(int argc, char* argv[]) {
    std::cout << "单关节控制器 - 交互模式\n";
    std::cout << "输入 help 查看帮助\n\n";

    // 设置实时权限（必须在其他线程创建之前调用）
    setup_realtime();

    // 初始化 Nanolog
    const char* home_dir = std::getenv("HOME");
    if (home_dir == nullptr) {
        home_dir = "/tmp";
    }
    std::filesystem::path log_dir = std::filesystem::path(home_dir) / "log";
    std::filesystem::create_directories(log_dir);
    nanolog::initialize(nanolog::GuaranteedLogger(), (log_dir / "").string(), "joint_control", 10);
    nanolog::set_log_level(nanolog::LogLevel::INFO);
    std::cout << "日志路径: " << (log_dir / "joint_control.txt").string() << "\n";

    // 初始化力传感器（可选，失败不影响运行）
    auto force_sensor = std::make_shared<SriForceSensor>();
    if (!force_sensor->Init(460800)) {
        std::cout << "力传感器初始化失败，将不记录力传感器数据\n";
        force_sensor = nullptr;  // 设为 nullptr，后续检查
    } else {
        std::cout << "力传感器初始化成功\n";
    }

    // 获取共享内存实例
    auto& fb_shm = FeedbackSharedMemory::GetInstance();
    auto& ctrl_shm = ControlSharedMemory::GetInstance();

    std::signal(SIGINT, signal_handler);

    // 1kHz 控制线程
    std::thread control_thread([&fb_shm, &ctrl_shm, &force_sensor]() {
        RobotFeedback feedback;
        RobotControl control;

        while (!g_stop_flag) {
            // 读取反馈获取状态字
            if (fb_shm.PeekLatest(&feedback)) {
                uint16_t status_word = feedback.robot_status_word;
                double current_pos = feedback.position[0];

                // 更新当前反馈数据
                current_position = current_pos;
                current_velocity = feedback.velocity[0];
                current_torque = feedback.torque[0];
                current_status_word = status_word;

                // 读取力传感器数据
                robot_server_lib::Vector6d force_data = robot_server_lib::Vector6d::Zero();
                if (force_sensor && force_sensor->PeekLatest(&force_data)) {
                    sensor_force = force_data;
                }

                // 记录数据
                if (enable_record) {
                    double timestamp_s =
                        static_cast<double>(std::chrono::steady_clock::now().time_since_epoch().count()) / 1e9;

                    LOG_INFO << "{\"timestamp\": " << std::to_string(timestamp_s) << ", "
                             << "\"position_cmd\": " << control.position[0] * 180.0 / M_PI << ", "
                             << "\"torque_cmd\": " << control.torque_feedforward[0] << ", "
                             << "\"position\": " << current_position * 180.0 / M_PI << ", "
                             << "\"velocity\": " << current_velocity * 180.0 / M_PI << ", "
                             << "\"torque\": " << current_torque << ", "
                             << "\"force\": [" << sensor_force[0] << ", " << sensor_force[1] << ", " << sensor_force[2]
                             << ", " << sensor_force[3] << ", " << sensor_force[4] << ", " << sensor_force[5] << "]"
                             << "}";
                }

                // 默认控制字：上电 + 清除故障
                control.robot_control_word = CTRLWORD_MOTOR_SWITCH_ON | CTRLWORD_MOTOR_CLEAR_FAULT;

                // 检测状态字：READY 或 ENABLED 后使能
                if (status_word == STATUSWORD_MOTOR_READY || status_word == STATUSWORD_MOTOR_ENABLED) {
                    control.robot_control_word |= CTRLWORD_MOTOR_ENABLE;
                }

                // 根据模式设置控制指令
                if (control_mode == "cst") {
                    // CST 力矩模式
                    control.robot_control_word |= CTRLWORD_MOTOR_CST_MODE;
                    control.torque_feedforward[0] = target_torque;
                }

                // 使能前：给定位置等于反馈位置，target position 也等于反馈位置
                if (status_word != STATUSWORD_MOTOR_READY && status_word != STATUSWORD_MOTOR_ENABLED) {
                    control.position[0] = current_pos;
                    target_position = current_pos;
                } else {
                    control.position[0] = target_position;
                }

                // 写入控制命令
                ctrl_shm.AppendNewElement(control);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // 主线程：处理用户输入（使用 readline 回调模式，支持 Ctrl+C 退出）
    print_help();

    // 安装 readline 回调
    rl_callback_handler_install("> ", input_callback);

    while (!g_stop_flag) {
        // 使用 select 检测输入
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;  // 50ms 超时

        int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            rl_callback_read_char();
        }

        // 检查是否有新输入
        if (g_got_input) {
            g_got_input = false;
            if (!g_input_line.empty()) {
                add_history(g_input_line.c_str());
                process_command(g_input_line);
            }
        }
    }

    // 清理 readline
    rl_callback_handler_remove();

    // 等待控制线程结束
    if (control_thread.joinable()) {
        control_thread.join();
    }

    std::cout << "程序退出\n";
    return 0;
}
