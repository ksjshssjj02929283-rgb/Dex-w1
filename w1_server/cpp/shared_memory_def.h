#pragma once

#include <pthread.h>
#include <sys/shm.h>
#include <time.h>
#include <atomic>
#include <iostream>

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

template <typename T, key_t SHM_KEY>
class SharedMemory
{
   public:
    static SharedMemory& GetInstance()
    {
        static SharedMemory instance;
        return instance;
    }
    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;

    // 设置超时时间（毫秒），-1表示不检测超时, 默认为100ms
    static void SetDataValidTimeoutMs(int64_t ms) { timeout_ms_.store(ms, std::memory_order_relaxed); }

    bool Write(const T& data) const
    {
        if (data_raw_ == nullptr)
        {
            std::cerr << "data_raw_ == nullptr" << std::endl;
            return false;
        }
        pthread_rwlock_wrlock(&data_raw_->rwlock);
        data_raw_->cnt++;
        data_raw_->data = data;
        clock_gettime(CLOCK_REALTIME, &data_raw_->ts);
        pthread_rwlock_unlock(&data_raw_->rwlock);
        return true;
    }

    bool Read(T* data) const
    {
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        if (data_raw_ == nullptr)
        {
            std::cerr << "data_raw_ == nullptr" << std::endl;
            return false;
        }

        int64_t timeout = timeout_ms_.load(std::memory_order_relaxed);
        // -1时不检测超时
        if (timeout != -1)
        {
            int64_t diff = (now.tv_sec - data_raw_->ts.tv_sec) * 1000LL + (now.tv_nsec - data_raw_->ts.tv_nsec) / 1000000LL;
            if (diff > timeout)
            {
                return false;
            }
        }

        pthread_rwlock_rdlock(&data_raw_->rwlock);
        *data = data_raw_->data;
        data->cnt = data_raw_->cnt;
        pthread_rwlock_unlock(&data_raw_->rwlock);
        return true;
    }

   protected:
    struct DataRaw
    {
        pthread_rwlock_t rwlock;
        uint64_t cnt = 0;
        struct timespec ts;
        T data;
    };

    DataRaw* const data_raw_ = nullptr;
    int shmid_ = -1;

    SharedMemory() : data_raw_(nullptr)
    {
        const int shm_size = sizeof(DataRaw);
        shmid_ = shmget(SHM_KEY, shm_size, IPC_CREAT | IPC_EXCL | 0666);
        int created = 0;
        if (shmid_ < 0)
        {
            if (errno == EEXIST)
            {
                shmid_ = shmget(SHM_KEY, shm_size, 0666);
                if (shmid_ < 0)
                {
                    std::cerr << "shmget" << std::endl;
                    return;
                }
                created = 0;
            }
            else
            {
                std::cerr << "shmget" << std::endl;
                return;
            }
        }
        else
        {
            created = 1;
        }
        DataRaw* tmp = (DataRaw*)shmat(shmid_, NULL, 0);
        if (tmp == (void*)(-1))
        {
            std::cerr << "shmat" << std::endl;
            return;
        }
        const_cast<DataRaw*&>(data_raw_) = tmp;
        if (created)
        {
            pthread_rwlockattr_t attr;
            pthread_rwlockattr_init(&attr);
            pthread_rwlockattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
            pthread_rwlock_init(&data_raw_->rwlock, &attr);
            pthread_rwlockattr_destroy(&attr);
        }
    }
    ~SharedMemory()
    {
        if (data_raw_ != nullptr)
        {
            shmdt(data_raw_);
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

    // 默认超时时间100ms，-1表示不检测超时
    static std::atomic<int64_t> timeout_ms_;
};

// 静态成员初始化
template <typename T, key_t SHM_KEY>
std::atomic<int64_t> SharedMemory<T, SHM_KEY>::timeout_ms_{100};

// 反馈共享内存单例
class FeedbackSharedMemory : public SharedMemory<RobotFeedback, FEEDBACK_SHM_KEY>
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
class ControlSharedMemory : public SharedMemory<RobotControl, CONTROL_SHM_KEY>
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