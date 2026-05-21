#ifndef SRI_FORCE_SERVER_H
#define SRI_FORCE_SERVER_H

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "NanoLog.hpp"
#include "single_rw_latest_queue.h"
#include "typedef.h"

typedef unsigned char BYTE;

#ifndef INT16
typedef short INT16;
#endif

#ifndef INT32
typedef int INT32;
#endif

#ifndef INT64
typedef long long INT64;
#endif

#ifndef byte
typedef unsigned char byte;
#endif

#define REC_DATA_LEN 8192

// 整合后的 SRI 力传感器类
class SriForceSensor {
public:
    SriForceSensor();
    ~SriForceSensor();

    // 初始化：自动查找串口、解析、零位校准
    bool Init(int baudrate = 460800);
    // 关闭串口
    void Close();

    // 读取最新力传感器数据 (fx, fy, fz, mx, my, mz)
    bool PeekLatest(robot_server_lib::Vector6d* output);
    bool PeekLatestRaw(robot_server_lib::Vector6d* output);
    bool GetZeroOffset(robot_server_lib::Vector6d* output) const;
    bool HasValidZeroOffset() const;
    int GetZeroOffsetSampleCount() const;

    // 设置零位校准时的工具自重补偿参数
    // rotation_world_from_ee_provider: 返回 R_world_from_EE 的旋转矩阵
    void SetToolGravityCompensationForZeroCalibration(
        bool enabled,
        double tool_mass_kg,
        const robot_server_lib::Vector3d& tool_com_m,
        double gravity_magnitude = 9.79,
        int model_sign = 1,
        std::function<std::optional<robot_server_lib::Matrix3d>()> rotation_world_from_ee_provider = nullptr);

    // 是否正在校准
    bool IsCalibrating() const;

private:
    // 串口相关
    int m_serial_fd;
    pthread_t m_rec_threadid;
    unsigned char* mReadBuffer;

    // 数据输出 buffer
    std::shared_ptr<robot_server_lib::SingleWriterLatestBuffer<robot_server_lib::Vector6d>> buffer_;
    std::shared_ptr<robot_server_lib::SingleWriterLatestBuffer<robot_server_lib::Vector6d>> raw_buffer_;

    // 零位校准相关（mZeroOffset 和 mCalibrationSum 均在 EE 坐标系下）
    bool mIsCalibrating = false;
    int mCalibrationSampleCount = 0;
    int mZeroOffsetSampleCount = 0;
    bool mZeroOffsetValid = false;
    float mZeroOffset[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float mCalibrationSum[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::chrono::steady_clock::time_point mCalibrationStartTime;

    bool mToolGravityCompensationForZeroEnabled = false;
    double mToolMassKg = 0.0;
    double mToolGravityMagnitude = 9.79;
    int mToolModelSign = 1;
    robot_server_lib::Vector3d mToolComM = robot_server_lib::Vector3d::Zero();
    std::function<std::optional<robot_server_lib::Matrix3d>()> mRotationWorldFromEEProvider;

    // 串口方法
    int OpenPort(const char* device);
    int SetPara(int serialfd, int speed);
    int WriteData(int fd, const char* data, int datalength);
    int ReadData(int fd, unsigned char* data, int datalength);
    void ClosePort(int fd);
    int BaudRate(int baudrate);

    // 读取线程
    static void* ReadThread(void* param);
    void ReadLoop();

    // 解析方法：成功返回新的偏移量，失败返回-1
    int ParseFrame(const unsigned char* data,
                   int dataLen,
                   int offset,
                   float& fx,
                   float& fy,
                   float& fz,
                   float& mx,
                   float& my,
                   float& mz);

    // CRC校验
    uint8_t MyCRC_GetCRC8(uint8_t* pDat, uint8_t bytes);

    // 零位校准方法
    void ProcessCalibrationData(float fx, float fy, float fz, float mx, float my, float mz);
    void ApplyZeroOffset(robot_server_lib::Vector6d& wrench_ee);
    static robot_server_lib::Vector6d MapSriRawToEE(float fx, float fy, float fz, float mx, float my, float mz);
    robot_server_lib::Vector6d ComputeToolGravityWrenchEE() const;

    // 辅助方法
    const char* FindSerialPort();
};

#endif
