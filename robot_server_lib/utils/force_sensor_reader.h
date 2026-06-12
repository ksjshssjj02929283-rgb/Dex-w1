#include <fcntl.h>
#include <poll.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

#include "typedef.h"

namespace robot_server_lib {

class ForceSensorReader {
public:
    explicit ForceSensorReader(const std::string& port, int baud = 921600)
        : port_(port), baud_(baud), fd_(-1), zero_calibrated_(false) {
        Initialize();
    }

    ~ForceSensorReader() {
        StopSensor();
        ClosePort();
    }

    // 初始化传感器并做零点校准100ms
    void Initialize() {
        if (this->ConfigurePort()) {
            this->StartSensor();
            if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
            // 零点校准线程避免阻塞
            std::thread([this]() {
                if (this->InitZeroCalibration(500)) {
                    sensor_ready_ = true;
                    std::cout << "Force sensor on port " << port_ << " is ready." << std::endl;
                }
            }).detach();
        } else {
            std::cout << "No force sensor in port: " << port_ << std::endl;
        }
    }

    bool ConfigurePort() {
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) return false;

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) return false;

        cfmakeraw(&tty);
#if defined(B921600)
        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);
#else
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
#endif
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS;

        // 非阻塞：由 O_NONBLOCK 控制等待，VMIN/VTIME 设为 0，避免 read 额外阻塞
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        tcflush(fd_, TCIOFLUSH);
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) return false;

        // 设置文件描述符为非阻塞
        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags == -1) return false;
        if (fcntl(fd_, F_SETFL, flags | O_NONBLOCK) == -1) return false;

        return true;
    }

    // 启动传感器采集
    bool StartSensor() {
        // 0x55 0xAA 0x58 0x01 0x02 0xFE: 开始采集
        uint8_t tx[6] = {0x55, 0xAA, 0x58, 0x01, 0x02, 0xFE};
        return SendCommand(tx, 6);
    }

    // 停止传感器采集
    bool StopSensor() {
        // 0x55 0xAA 0x58 0x01 0x03 0xFD: 停止采集
        uint8_t tx[6] = {0x55, 0xAA, 0x58, 0x01, 0x03, 0xFD};
        return SendCommand(tx, 6);
    }

    // 初始化并做零点校准（收集 count 帧取均值）
    bool InitZeroCalibration(int count = 500) {
        std::vector<std::array<double, 6>> buf;
        buf.reserve(count);
        int attempts = 0;
        while ((int)buf.size() < count && attempts < count * 3) {
            std::array<double, 6> darr;
            if (ReadFrameOnce(darr)) {
                buf.push_back(darr);
            }
            ++attempts;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (buf.empty()) {
            std::cout << "Failed to calibrate force sensor in port " << port_ << ", no sensor data." << std::endl;
            return false;
        }

        std::array<double, 6> mean{};
        for (int i = 0; i < 6; ++i) {
            double s = 0;
            for (auto& r : buf) s += r[i];
            mean[i] = s / buf.size();
        }
        zero_frame_ = mean;
        zero_calibrated_ = true;
        return true;
    }

    // 尝试读取一帧（非阻塞 + 整体超时，避免死等）
    // M4313SXX 数据帧格式: 0x55 0xAA | 0x14 | ID | Fx(2) | Fy(2) | Fz(2) | Mx(2) | My(2) | Mz(2) | Status(2) | Sum |
    // 0x0A 共 20 字节 (0x14 = 20)
    bool ReadFrameOnce(std::array<double, 6>& out, int attempts = 3) {
        if (fd_ < 0) return false;

        uint8_t buf[32];

        for (int attempt = 0; attempt < attempts; ++attempt) {
            // 等待读事件，最久 2ms，避免 busy-loop
            struct pollfd pfd {
                fd_, POLLIN, 0
            };
            int pr = poll(&pfd, 1, 2);
            if (pr <= 0) {
                // 超时或被信号打断，下一次尝试
                continue;
            }

            // 读首字节，查找帧头 0x55 0xAA
            uint8_t b = 0;
            ssize_t r = ::read(fd_, &b, 1);
            if (r < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                // 其他错误
                return false;
            }
            if (r == 0) continue;

            if (b != 0x55) {
                // 非帧头，继续寻找
                continue;
            }

            buf[0] = b;

            // 读取第二字节，必须是 0xAA
            r = ::read(fd_, &b, 1);
            if (r != 1 || b != 0xAA) {
                // 不是 0xAA，重新同步
                continue;
            }
            buf[1] = 0xAA;

            // 读取剩余 18 字节（总共 20 字节），设置整体 2ms deadline
            auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
            ssize_t got = 0;
            while (got < 18) {
                auto now = std::chrono::steady_clock::now();
                if (now >= deadline) {
                    // 放弃当前帧并清理缓冲以重新同步
                    tcflush(fd_, TCIFLUSH);
                    break;
                }

                int remain = 18 - got;
                int wait_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
                if (wait_ms > 2) wait_ms = 2;  // 小步等待

                struct pollfd p2 {
                    fd_, POLLIN, 0
                };
                int pr2 = poll(&p2, 1, wait_ms);
                if (pr2 <= 0) {
                    // 超时或被信号打断，继续循环检查整体超时
                    continue;
                }

                ssize_t rr = ::read(fd_, buf + 2 + got, remain);
                if (rr > 0) {
                    got += rr;
                    continue;
                }
                if (rr < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    // 还没数据，继续等
                    continue;
                }
                // 其他错误或异常，放弃该帧
                break;
            }

            if (got < 18) {
                // 本次帧不完整
                continue;
            }

            // 验证帧尾 0x0A
            if (buf[19] != 0x0A) {
                // 帧尾不对，尝试下一帧
                continue;
            }

            // 校验累加和（字节 2-18 的累加，结果取低 8 位）
            uint8_t sum = 0;
            for (int i = 2; i < 19; ++i) {
                sum += buf[i];
            }
            if (sum != buf[18]) {
                // 校验失败
                continue;
            }

            // 解析 6 个 int16（小端，偏移 3 开始，每通道 2 字节）
            // Fx, Fy, Fz, Mx, My, Mz
            for (int i = 0; i < 6; ++i) {
                int16_t v;
                std::memcpy(&v, buf + 3 + i * 2, 2);
                out[i] = static_cast<double>(v);
            }
            return true;
        }
        return false;
    }

    // 非阻塞读取并自动做零点补偿，返回是否有新数据
    bool Read(Vector6d& out) {
        if (!zero_calibrated_ || !sensor_ready_) return false;
        std::array<double, 6> tmp{};
        if (!ReadFrameOnce(tmp)) return false;
        for (int i = 0; i < 6; ++i) out(i) = tmp[i] - zero_frame_[i];
        return true;
    }

private:
    bool SendCommand(const uint8_t* cmd, size_t len) {
        if (fd_ < 0) return false;
        ssize_t w = ::write(fd_, cmd, len);
        tcdrain(fd_);
        return w == (ssize_t)len;
    }

    void ClosePort() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    std::string port_;
    int baud_;
    int fd_;
    bool zero_calibrated_;
    bool sensor_ready_ = false;
    std::array<double, 6> zero_frame_;
};

}  // namespace robot_server_lib