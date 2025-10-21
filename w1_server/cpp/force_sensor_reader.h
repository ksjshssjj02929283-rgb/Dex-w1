#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <array>
#include <vector>
#include <numeric>
#include "robot_server_lib/typedef.h"

using namespace robot_server_lib;

class ForceSensorReader
{
public:
    explicit ForceSensorReader(const std::string &port, int baud = 460800)
        : port_(port), baud_(baud), fd_(-1), zero_calibrated_(false), frame_count_(0)
    {
    }

    ~ForceSensorReader()
    {
        StopSensor();
        ClosePort();
    }

    // 初始化传感器并做零点校准100ms
    bool Initialize()
    {
        bool sensor_ready = false;
        if (this->ConfigurePort())
        {
            this->StartSensor();
            if (this->InitZeroCalibration(100))
            {
                sensor_ready = true;
            }
        }
        return sensor_ready;
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
        // 如果系统不支持 B460800，请根据环境设置
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
#endif
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; // 0.1s

        tcflush(fd_, TCIOFLUSH);
        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            return false;

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
    bool InitZeroCalibration(int count = 1000)
    {
        if (fd_ < 0)
            if (!ConfigurePort())
                return false;
        if (!StartSensor())
            return false;

        std::vector<std::array<double, 6>> buf;
        buf.reserve(count);
        int attempts = 0;
        while ((int)buf.size() < count && attempts < count * 5)
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
            return false;

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

    // 尝试读取一帧，返回 true 并填充 out（单位：SI，与 Python 保持一致）
    bool ReadFrameOnce(std::array<double, 6> &out, double timeout_s = 0.01)
    {
        if (fd_ < 0)
            return false;

        uint8_t buf[28];
        ssize_t n = 0;
        auto start = std::chrono::steady_clock::now();
        // 读取第一个字节（过滤非 0x48/0x49）
        while (true)
        {
            uint8_t b;
            ssize_t r = ::read(fd_, &b, 1);
            if (r == 1)
            {
                if (b == 0x48 || b == 0x49)
                {
                    buf[0] = b;
                    // 然后读取剩余的 27 字节
                    ssize_t got = 0;
                    while (got < 27)
                    {
                        ssize_t rr = ::read(fd_, buf + 1 + got, 27 - got);
                        if (rr > 0)
                            got += rr;
                        else
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                    // 验证尾部 CR LF
                    if (buf[26] != 0x0D || buf[27] != 0x0A)
                        break;
                    // 解析 6 个 float（小端，偏移 2 开始）
                    for (int i = 0; i < 6; ++i)
                    {
                        float v;
                        std::memcpy(&v, buf + 2 + i * 4, 4);
                        out[i] = static_cast<double>(v) * 9.80665; // 与 Python 保持一致
                    }
                    ++frame_count_;
                    return true;
                }
            }
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > timeout_s)
                return false;
        }
        return false;
    }

    // 非阻塞读取并自动做零点补偿，返回是否有新数据
    bool Read(Vector6d &out)
    {
        if (!zero_calibrated_)
            return false;
        std::array<double, 6> tmp{};
        if (!ReadFrameOnce(tmp))
            return false;
        // 零点校准补偿并转换为 Eigen 向量
        for (int i = 0; i < 6; ++i)
            out(i) = tmp[i] - zero_frame_[i];
        out(0) = -out(0);
        out(5) = -out(5);
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
    std::array<double, 6> zero_frame_;
    int frame_count_;
};