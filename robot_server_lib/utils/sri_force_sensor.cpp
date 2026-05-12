#include "sri_force_sensor.h"

#include <pthread.h>

void Setup_Realtime(int priority) {
    // 锁内存，避免分页抖动
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "mlockall failed: " << std::strerror(errno) << std::endl;
    }
    // 设置SCHED_FIFO实时调度
    sched_param sp{};
    sp.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::cerr << "sched_setscheduler failed: " << std::strerror(errno) << std::endl;
        LOG_CRIT << "sched_setscheduler failed, please run setcap!";
        assert(false && "sched_setscheduler failed, please run setcap!");
    }
}

//====================================
// SriForceSensor Implementation
//====================================

SriForceSensor::SriForceSensor() : m_serial_fd(-1), m_rec_threadid(0), mReadBuffer(nullptr) {
    mReadBuffer = new unsigned char[REC_DATA_LEN];
}

SriForceSensor::~SriForceSensor() {
    Close();
    delete[] mReadBuffer;
}

// 串口方法
const char* SriForceSensor::FindSerialPort() {
    const char* candidates[] = {"/dev/ttyUSB2"};
    for (const auto& device : candidates) {
        if (access(device, F_OK) == 0) {
            printf("Found SRI force sensor at: %s\n", device);
            return device;
        }
    }
    return nullptr;
}

int SriForceSensor::OpenPort(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    return fd;
}

int SriForceSensor::BaudRate(int baudrate) {
    switch (baudrate) {
        case 115200:
            return B115200;
        case 460800:
            return B460800;
        default:
            return B9600;
    }
}

int SriForceSensor::SetPara(int serialfd, int speed) {
    struct termios tty;
    bzero(&tty, sizeof(tty));
    cfmakeraw(&tty);
    tty.c_cflag = BaudRate(speed) | CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~PARODD;
    tty.c_cflag &= ~CSTOPB;
    tcflush(serialfd, TCIFLUSH);
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 1;
    return tcsetattr(serialfd, TCSANOW, &tty);
}

int SriForceSensor::WriteData(int fd, const char* data, int datalength) {
    if (fd < 0) return -1;

    int len = 0, total_len = 0;
    while (total_len < datalength) {
        len = write(fd, &data[total_len], datalength - total_len);
        if (len > 0) {
            total_len += len;
        } else {
            return -1;
        }
    }
    return total_len;
}

int SriForceSensor::ReadData(int fd, unsigned char* data, int datalength) {
    if (fd < 0) return -1;

    fd_set readset;
    FD_ZERO(&readset);
    FD_SET(fd, &readset);

    struct timeval tv = {0, 20000};
    if (select(fd + 1, &readset, nullptr, nullptr, &tv) <= 0) {
        return 0;
    }
    return FD_ISSET(fd, &readset) ? read(fd, data, datalength) : 0;
}

void SriForceSensor::ClosePort(int fd) {
    if (fd > 0) {
        ::close(fd);
    }
}

// 零位校准方法
void SriForceSensor::ProcessCalibrationData(float fx, float fy, float fz, float mx, float my, float mz) {
    if (!mIsCalibrating) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - mCalibrationStartTime).count();

    if (elapsed >= 500) {
        if (mCalibrationSampleCount > 0) {
            for (int i = 0; i < 6; i++) {
                mZeroOffset[i] = mCalibrationSum[i] / mCalibrationSampleCount;
            }
            printf(
                "[Info] SRI force sensor: Zero calibration completed: samples=%d, offset=%.4f, %.4f, %.4f, %.4f, %.4f, "
                "%.4f\n",
                mCalibrationSampleCount, mZeroOffset[0], mZeroOffset[1], mZeroOffset[2], mZeroOffset[3], mZeroOffset[4],
                mZeroOffset[5]);
            LOG_INFO << "Zero calibration completed: samples=" << mCalibrationSampleCount
                     << ", offset=" << mZeroOffset[0] << ", " << mZeroOffset[1] << ", " << mZeroOffset[2] << ", "
                     << mZeroOffset[3] << ", " << mZeroOffset[4] << ", " << mZeroOffset[5];
        }
        mIsCalibrating = false;
        return;
    }

    mCalibrationSum[0] += fx;
    mCalibrationSum[1] += fy;
    mCalibrationSum[2] += fz;
    mCalibrationSum[3] += mx;
    mCalibrationSum[4] += my;
    mCalibrationSum[5] += mz;
    mCalibrationSampleCount++;
}

void SriForceSensor::ApplyZeroOffset(float& fx, float& fy, float& fz, float& mx, float& my, float& mz) {
    fx -= mZeroOffset[0];
    fy -= mZeroOffset[1];
    fz -= mZeroOffset[2];
    mx -= mZeroOffset[3];
    my -= mZeroOffset[4];
    mz -= mZeroOffset[5];
}

// 解析方法
uint8_t SriForceSensor::MyCRC_GetCRC8(uint8_t* pDat, uint8_t bytes) {
    uint8_t CRC8_POLY = 0x8C;
    uint8_t crc = 0;
    while (bytes--) {
        crc ^= *pDat;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ CRC8_POLY;
            } else {
                crc >>= 1;
            }
        }
        pDat++;
    }
    return crc;
}

int SriForceSensor::ParseFrame(const unsigned char* data,
                               int dataLen,
                               int offset,
                               float& fx,
                               float& fy,
                               float& fz,
                               float& mx,
                               float& my,
                               float& mz) {
    if (offset + 14 > dataLen) {
        return -1;
    }

    // 查找帧头 0xAA 0x55
    int headIndex = -1;
    for (int i = offset; i < dataLen - 1; i++) {
        if (data[i] == 0xAA && data[i + 1] == 0x55) {
            headIndex = i;
            break;
        }
    }
    if (headIndex == -1 || headIndex + 14 > dataLen) {
        return -1;
    }

    // 解析数据
    int index = headIndex + 2;
    byte buf[11];
    memcpy(&buf, data + index, 11);

    INT32 forceInt32[6];
    forceInt32[0] = (buf[0] << 9) + (buf[1] << 1) + ((buf[2] & 0x80) >> 7);
    forceInt32[1] = ((buf[2] & 0x7F) << 10) + ((buf[3] & 0xFF) << 2) + ((buf[4] & 0xC0) >> 6);
    forceInt32[2] = ((buf[4] & 0x3F) << 11) + ((buf[5] & 0xFF) << 3) + ((buf[6] & 0xE0) >> 5);
    forceInt32[3] = ((buf[6] & 0x1F) << 7) + ((buf[7] & 0xFE) >> 1);
    forceInt32[4] = ((buf[7] & 0x01) << 11) + ((buf[8] & 0xFF) << 3) + ((buf[9] & 0xE0) >> 5);
    forceInt32[5] = ((buf[9] & 0x1F) << 7) + ((buf[10] & 0xFE) >> 1);

    BYTE crc = data[index + 11];
    BYTE calCrc = MyCRC_GetCRC8(buf, 11);
    if (crc != calCrc) {
        return headIndex + 1;  // CRC失败，跳过1字节
    }

    float f[6];
    for (int i = 0; i < 3; i++) {
        f[i] = (forceInt32[i] & 0x10000) ? -(float)((forceInt32[i] & 0xffff) / 100.0)
                                         : (float)((forceInt32[i] & 0xffff) / 100.0);
    }
    for (int i = 3; i < 6; i++) {
        f[i] = (forceInt32[i] & 0x0800) ? -(float)((forceInt32[i] & 0x07ff) / 100.0)
                                        : (float)((forceInt32[i] & 0x07ff) / 100.0);
    }

    fx = f[0];
    fy = f[1];
    fz = f[2];
    mx = f[3];
    my = f[4];
    mz = f[5];
    return headIndex + 14;  // 返回解析成功后的新偏移量
}

// 读取线程
void* SriForceSensor::ReadThread(void* param) {
    Setup_Realtime(70);
    auto* sensor = static_cast<SriForceSensor*>(param);
    sensor->ReadLoop();
    pthread_detach(pthread_self());
    pthread_exit(nullptr);
    return nullptr;
}

void SriForceSensor::ReadLoop() {
    printf("[Info] SRI force sensor: Read thread started\n");
    LOG_INFO << "SRI force sensor: Read thread started";
    auto last_abnormal_log_time = std::chrono::steady_clock::now() - std::chrono::seconds(5);
    auto log_read_abnormal = [&last_abnormal_log_time]() {
        auto now = std::chrono::steady_clock::now();
        if (now - last_abnormal_log_time >= std::chrono::seconds(1)) {
            printf("[Warn] Force sensor data read abnormal\n");
            LOG_WARN << "Force sensor data read abnormal";
            last_abnormal_log_time = now;
        }
    };
    std::vector<unsigned char> rx_buffer;
    rx_buffer.reserve(REC_DATA_LEN);
    const unsigned char frame_header[] = {0xAA, 0x55};
    while (true) {
        if (m_serial_fd < 0) break;

        int recvDataLen = ReadData(m_serial_fd, mReadBuffer, REC_DATA_LEN);
        if (recvDataLen > 0) {
            rx_buffer.insert(rx_buffer.end(), mReadBuffer, mReadBuffer + recvDataLen);

            if (rx_buffer.size() > REC_DATA_LEN) {
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.end() - 1024);
            }

            float fx = 0, fy = 0, fz = 0, mx = 0, my = 0, mz = 0;
            bool found = false;

            while (true) {
                auto header = std::search(rx_buffer.begin(), rx_buffer.end(), frame_header, frame_header + 2);
                if (header == rx_buffer.end()) {
                    if (rx_buffer.size() > 1) {
                        rx_buffer.erase(rx_buffer.begin(), rx_buffer.end() - 1);
                    }
                    break;
                }

                if (header != rx_buffer.begin()) {
                    rx_buffer.erase(rx_buffer.begin(), header);
                }

                if (rx_buffer.size() < 14) {
                    break;
                }

                float tfx, tfy, tfz, tmx, tmy, tmz;
                int newOffset = ParseFrame(rx_buffer.data(), static_cast<int>(rx_buffer.size()), 0, tfx, tfy, tfz, tmx, tmy, tmz);
                if (newOffset == 14) {
                    fx = tfx;
                    fy = tfy;
                    fz = tfz;
                    mx = tmx;
                    my = tmy;
                    mz = tmz;
                    found = true;
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 14);
                } else if (newOffset > 0) {
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 1);
                } else {
                    break;
                }
            }

            if (found) {
                if (mIsCalibrating) {
                    ProcessCalibrationData(fx, fy, fz, mx, my, mz);
                } else {
                    robot_server_lib::Vector6d raw_vec;
                    raw_vec << fx, fy, fz, mx, my, mz;
                    if (raw_buffer_) {
                        raw_buffer_->AppendNewElement(raw_vec);
                    }

                    ApplyZeroOffset(fx, fy, fz, mx, my, mz);
                    if (buffer_) {
                        robot_server_lib::Vector6d vec;
                        vec << fx, fy, fz, mx, my, mz;
                        buffer_->AppendNewElement(vec);
                    }
                }
            } else if (rx_buffer.size() >= 14) {
                log_read_abnormal();
            }
        } else {
            log_read_abnormal();
            usleep(50);
        }
    }
    printf("[Info] SRI force sensor: Read thread stopped\n");
    LOG_INFO << "SRI force sensor: Read thread stopped";
}

bool SriForceSensor::Init(int baudrate) {
    // 查找串口
    const char* port = FindSerialPort();
    if (port == nullptr) {
        printf("[Warn] SRI force sensor: No sensor found!\n");
        LOG_WARN << "No SRI force sensor found!";
        return false;
    }

    // 创建 buffer
    buffer_ = std::make_shared<robot_server_lib::SingleWriterLatestBuffer<robot_server_lib::Vector6d>>();
    raw_buffer_ = std::make_shared<robot_server_lib::SingleWriterLatestBuffer<robot_server_lib::Vector6d>>();

    // 打开串口
    m_serial_fd = OpenPort(port);
    if (m_serial_fd < 0) {
        printf("[Warn] SRI force sensor: Failed to open serial port: %s\n", port);
        LOG_WARN << "Failed to open serial port: " << port;
        return false;
    }
    printf("[Info] SRI force sensor: Opened serial port %d\n", m_serial_fd);

    // 配置串口参数
    SetPara(m_serial_fd, baudrate);

    // 启动读取线程
    if (pthread_create(&m_rec_threadid, nullptr, ReadThread, this) != 0) {
        printf("[Warn] SRI force sensor: Failed to create read thread\n");
        LOG_WARN << "Failed to create read thread";
        ClosePort(m_serial_fd);
        m_serial_fd = -1;
        return false;
    }

    // 等待数据开始流动
    usleep(100000);

    // 启动零位校准
    mIsCalibrating = false;
    mCalibrationSampleCount = 0;
    memset(mCalibrationSum, 0, sizeof(mCalibrationSum));
    mCalibrationStartTime = std::chrono::steady_clock::now();

    // 等待校准完成
    while (mIsCalibrating) {
        usleep(10000);
    }

    printf("[Info] SRI force sensor: Initialized successfully\n");
    LOG_INFO << "SRI force sensor initialized successfully";
    return true;
}

void SriForceSensor::Close() {
    if (m_serial_fd > 0) {
        ClosePort(m_serial_fd);
        m_serial_fd = -1;
    }
    if (m_rec_threadid) {
        pthread_cancel(m_rec_threadid);
        m_rec_threadid = 0;
    }
}

bool SriForceSensor::PeekLatest(robot_server_lib::Vector6d* output) {
    if (buffer_ && output) {
        return buffer_->PeekLatest(output);
    }
    return false;
}

bool SriForceSensor::PeekLatestRaw(robot_server_lib::Vector6d* output) {
    if (raw_buffer_ && output) {
        return raw_buffer_->PeekLatest(output);
    }
    return false;
}

bool SriForceSensor::IsCalibrating() const { return mIsCalibrating; }
