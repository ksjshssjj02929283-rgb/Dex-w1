#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <array>
#include <vector>
#include <numeric>
#include "robot_server_lib/typedef.h"
#include <thread>
#include <chrono>
#include <cerrno>
#include <poll.h>

using namespace robot_server_lib;

class ForceSensorReader
{
public:
    explicit ForceSensorReader(const std::string &port, int baud = 460800)
        : port_(port), baud_(baud), fd_(-1), zero_calibrated_(false)
    {
        Initialize();
    }

    ~ForceSensorReader()
    {
        StopSensor();
        ClosePort();
    }

    // 初始化传感器并做零点校准100ms
    void Initialize()
    {
        if (this->ConfigurePort())
        {
            this->StartSensor();
            if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
            // 零点校准线程避免阻塞
            std::thread([this]() 
            {
                if (this->InitZeroCalibration(500))
                {
                    sensor_ready_ = true;
                }
            }).detach();
        }
    }

    bool ConfigurePort()
    {
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
            return false;

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0)
            return false;

        cfmakeraw(&tty);
#if defined(B460800)
        cfsetospeed(&tty, B460800);
        cfsetispeed(&tty, B460800);
#else
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
#endif
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS;

        // 非阻塞：由 O_NONBLOCK 控制等待，VMIN/VTIME 设为 0，避免 read 额外阻塞
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        tcflush(fd_, TCIOFLUSH);
        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            return false;

        // 设置文件描述符为非阻塞
        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags == -1) return false;
        if (fcntl(fd_, F_SETFL, flags | O_NONBLOCK) == -1) return false;

        return true;
    }

    bool StartSensor()
    {
        return SendCommand(0x48);
    }

    bool StopSensor()
    {
        return SendCommand(0x43);
    }

    // 初始化并做零点校准（收集 count 帧取均值）
    bool InitZeroCalibration(int count = 500)
    {
        std::vector<std::array<double, 6>> buf;
        buf.reserve(count);
        int attempts = 0;
        while ((int)buf.size() < count && attempts < count * 3)
        {
            std::array<double, 6> darr;
            if (ReadFrameOnce(darr))
            {
                buf.push_back(darr);
            }
            ++attempts;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (buf.empty())
        {
            std::cout << "Failed to calibrate force sensor in port " << port_ << ", no sensor data." << std::endl;
            return false;
        }

        std::array<double, 6> mean{};
        for (int i = 0; i < 6; ++i)
        {
            double s = 0;
            for (auto &r : buf)
                s += r[i];
            mean[i] = s / buf.size();
        }
        zero_frame_ = mean;
        zero_calibrated_ = true;
        return true;
    }

    // 尝试读取一帧（非阻塞 + 整体超时，避免死等）
    bool ReadFrameOnce(std::array<double, 6> &out, int attempts = 3)
    {
        if (fd_ < 0)
            return false;

        uint8_t buf[28];

        for (int attempt = 0; attempt < attempts; ++attempt)
        {
            // 等待读事件，最久 2ms，避免 busy-loop
            struct pollfd pfd{fd_, POLLIN, 0};
            int pr = poll(&pfd, 1, 2);
            if (pr <= 0)
            {
                // 超时或被信号打断，下一次尝试
                continue;
            }

            // 读首字节
            uint8_t b = 0;
            ssize_t r = ::read(fd_, &b, 1);
            if (r < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    continue;
                // 其他错误
                return false;
            }
            if (r == 0)
                continue;

            if (b != 0x48 && b != 0x49)
            {
                // 非起始字节，丢弃继续
                continue;
            }

            buf[0] = b;

            // 读取剩余 27 字节，设置整体 2ms deadline
            auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
            ssize_t got = 0;
            while (got < 27)
            {
                auto now = std::chrono::steady_clock::now();
                if (now >= deadline)
                {
                    // 放弃当前帧并清理缓冲以重新同步
                    tcflush(fd_, TCIFLUSH);
                    break;
                }

                int remain = 27 - got;
                int wait_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
                if (wait_ms > 2) wait_ms = 2; // 小步等待

                struct pollfd p2{fd_, POLLIN, 0};
                int pr2 = poll(&p2, 1, wait_ms);
                if (pr2 <= 0)
                {
                    // 超时或被信号打断，继续循环检查整体超时
                    continue;
                }

                ssize_t rr = ::read(fd_, buf + 1 + got, remain);
                if (rr > 0)
                {
                    got += rr;
                    continue;
                }
                if (rr < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                {
                    // 还没数据，继续等
                    continue;
                }
                // 其他错误或异常，放弃该帧
                break;
            }

            if (got < 27)
            {
                // 本次帧不完整
                continue;
            }

            // 验证尾部 CR LF
            if (buf[26] != 0x0D || buf[27] != 0x0A)
            {
                // 尾部不对，尝试下一帧
                continue;
            }

            // 解析 6 个 float（小端，偏移 2 开始）
            for (int i = 0; i < 6; ++i)
            {
                float v;
                std::memcpy(&v, buf + 2 + i * 4, 4);
                out[i] = static_cast<double>(v) * 9.80665;
            }
            return true;
        }
        return false;
    }

    // 非阻塞读取并自动做零点补偿，返回是否有新数据
    bool Read(Vector6d &out)
    {
        out = Vector6d::Zero();
        if (!zero_calibrated_ || !sensor_ready_)
            return false;
        std::array<double, 6> tmp{};
        if (!ReadFrameOnce(tmp))
            return false;
        for (int i = 0; i < 6; ++i)
            out(i) = tmp[i] - zero_frame_[i];
        return true;
    }

private:
    bool SendCommand(uint8_t cmd)
    {
        if (fd_ < 0)
            return false;
        uint8_t tx[4] = {cmd, 0xAA, 0x0D, 0x0A};
        ssize_t w = ::write(fd_, tx, 4);
        tcdrain(fd_);
        return w == 4;
    }

    void ClosePort()
    {
        if (fd_ >= 0)
        {
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