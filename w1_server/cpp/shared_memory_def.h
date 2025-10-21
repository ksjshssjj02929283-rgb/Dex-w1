#pragma once

#include <array>
#include <atomic>
#include <sys/shm.h>
#include <cassert>
#include <cstring>
#include <iostream>
#include <new>    
#include <cstdint>   
#include <cerrno>     

#define FEEDBACK_SHM_KEY 0x1000
#define CONTROL_SHM_KEY 0x2000
#define MAX_JOINT_NUM 32

struct RobotFeedback
{
    uint64_t cnt = 0;
    uint16_t robot_status_word = 0x0000;
    uint16_t robot_error_code = 0x0000;
    double position[MAX_JOINT_NUM] = {0};
    double velocity[MAX_JOINT_NUM] = {0};
    double torque[MAX_JOINT_NUM] = {0};
};

struct RobotControl
{
    uint64_t cnt = 0;
    uint16_t robot_control_word = 0x0000;
    double position[MAX_JOINT_NUM] = {0};
    double velocity[MAX_JOINT_NUM] = {0};
    double acceleration[MAX_JOINT_NUM] = {0};
    double torque_feedforward[MAX_JOINT_NUM] = {0};
    double torque_external[MAX_JOINT_NUM] = {0};

    double positive_velocity_limit[MAX_JOINT_NUM] = {0};
    double negative_velocity_limit[MAX_JOINT_NUM] = {0};
    double positive_torque_limit[MAX_JOINT_NUM] = {0};
    double negative_torque_limit[MAX_JOINT_NUM] = {0};

    int chassis_mode = 0;
    int chassis_whole_mode = 0;
    double chassis_velocity = 0.0;
    double chassis_direction = 0.0;
    double chassis_left_velocity = 0.0;
    double chassis_right_velocity = 0.0;
};

inline void decodeTwoUint32FromUint64(std::uint64_t v, std::uint32_t &latest, std::uint32_t &reader) {
    latest = static_cast<std::uint32_t>(v >> 32);
    reader = static_cast<std::uint32_t>(v & 0xFFFFFFFFu);
}

inline std::uint64_t encodeTwoUint32AsUint64(std::uint32_t latest, std::uint32_t reader) {
    return (static_cast<std::uint64_t>(latest) << 32) | reader;
}

template <typename T, std::uint64_t BufferCapacity, key_t SHM_KEY>
class SharedMemory
{
   public:
    static SharedMemory& GetInstance()
    {
        static SharedMemory instance;
        return instance;
    }

    bool AppendNewElement(const T &data);
    bool PeekLatest(T *data);
    bool is_valid() const { return valid_; }
    // 设置超时时间（毫秒），-1表示不检测超时, 默认为100ms
    static void SetDataValidTimeoutMs(int64_t ms) { timeout_ms_.store(ms, std::memory_order_relaxed); }
    
   protected:
    struct QueueElement
    {
        T data;
        uint64_t cnt = 0;
        struct timespec ts;
        bool valid{false};  // Handle the case if no element has arrived yet
    };

    struct ShmStruct
    {
        QueueElement ring_buffer[BufferCapacity];
        std::atomic_uint64_t latest_and_reader;
    };

    ShmStruct* shm_ptr_;
    int shmid_ = -1;
    bool valid_ = false;
    int64_t cnt_ = 0;
    static std::atomic<int64_t> timeout_ms_;
    static constexpr std::uint32_t kInvalidReader = 0xFFFFFFFFu;

    SharedMemory() : shm_ptr_(nullptr) {
        shmid_ = shmget(SHM_KEY, sizeof(ShmStruct), IPC_CREAT | IPC_EXCL | 0666);
        bool created = false;
        if (shmid_ < 0) {
            if (errno == EEXIST) {
                shmid_ = shmget(SHM_KEY, sizeof(ShmStruct), 0666);
                if (shmid_ < 0) {
                    std::cerr << "shmget failed, errno=" << errno << " (" << strerror(errno) << ")\n";
                    valid_ = false;
                    return;
                }
            } else {
                std::cerr << "shmget failed, errno=" << errno << " (" << strerror(errno) << ")\n";
                valid_ = false;
                return;
            }
        } else {
            created = true;
        }
        void* tmp = shmat(shmid_, NULL, 0);
        if (tmp == (void*)(-1)) {
            std::cerr << "shmat failed, errno=" << errno << " (" << strerror(errno) << ")\n";
            valid_ = false;
            return;
        }
        shm_ptr_ = reinterpret_cast<ShmStruct*>(tmp);
        if (created) {
            // 在共享内存中值初始化构造对象
            new (shm_ptr_) ShmStruct();

            // 设置 latest/reader 初值（避免 0 的歧义，显式使用 0u）
            shm_ptr_->latest_and_reader.store(
                encodeTwoUint32AsUint64(0u, kInvalidReader),
                std::memory_order_relaxed
            );
        }
        valid_ = true;
    }

    ~SharedMemory() {
        if (shm_ptr_) {
            shmdt(shm_ptr_);
        }
        if (shmid_ != -1)
        {
            struct shmid_ds buf;
            shmctl(shmid_, IPC_STAT, &buf);
            if (buf.shm_nattch == 0)
            {
                std::cout << "buf.shm_nattch = " << buf.shm_nattch << " Shared memory segment removed." << std::endl;
                shmctl(shmid_, IPC_RMID, NULL);
            }
            else
            {
                std::cout << "buf.shm_nattch = " << buf.shm_nattch << "." << " Shared memory segment still in use." << std::endl;
            }
        }
    }

    void tryWrapLatestAndReader(std::uint32_t &latest, std::uint32_t &reader);
};

// 静态成员初始化
template <typename T, std::uint64_t BufferCapacity, key_t SHM_KEY>
std::atomic<int64_t> SharedMemory<T, BufferCapacity, SHM_KEY>::timeout_ms_{100};

// 反馈共享内存单例
class FeedbackSharedMemory : public SharedMemory<RobotFeedback, 8, FEEDBACK_SHM_KEY>
{
   public:
    static FeedbackSharedMemory& GetInstance()
    {
        static FeedbackSharedMemory instance;
        return instance;
    }
    FeedbackSharedMemory(const FeedbackSharedMemory&) = delete;
    FeedbackSharedMemory& operator=(const FeedbackSharedMemory&) = delete;

   private:
    FeedbackSharedMemory() : SharedMemory() {}  // 显式声明默认构造函数
};

// 控制共享内存单例
class ControlSharedMemory : public SharedMemory<RobotControl, 8, CONTROL_SHM_KEY>
{
   public:
    static ControlSharedMemory& GetInstance()
    {
        static ControlSharedMemory instance;
        return instance;
    }
    ControlSharedMemory(const ControlSharedMemory&) = delete;
    ControlSharedMemory& operator=(const ControlSharedMemory&) = delete;

   private:
    ControlSharedMemory() : SharedMemory() {}  // 显式声明默认构造函数
};

#include "shared_memory_def.hpp"