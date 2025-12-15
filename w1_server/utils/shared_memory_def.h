#pragma once

#include <sys/shm.h>

#include <array>
#include <atomic>
#include <cassert>
#include <cstring>
#include <iostream>

#define FEEDBACK_SHM_KEY 0x1000
#define CONTROL_SHM_KEY 0x2000
#define MAX_JOINT_NUM 32
#define OMNI_MOTOR_NUM 4

struct ChassisFeedback
{
    int64_t timestamp;
    double left_velocity;
    double right_velocity;
};

struct RobotFeedback
{
    uint64_t cnt = 0;
    uint16_t robot_status_word = 0x0000;
    uint8_t al_status[MAX_JOINT_NUM] = {0};
    uint16_t motor_error_code[MAX_JOINT_NUM] = {0};
    uint8_t motor_temperature[MAX_JOINT_NUM] = {0};
    uint16_t robot_error_code = 0x0000;
    // 关节反馈
    double position[MAX_JOINT_NUM] = {0};
    double velocity[MAX_JOINT_NUM] = {0};
    double torque[MAX_JOINT_NUM] = {0};
    // 底盘反馈
    ChassisFeedback chassis_feedback;
};

struct RobotControl
{
    uint64_t cnt = 0;
    uint16_t robot_control_word = 0x0000;
    // 关节控制
    double position[MAX_JOINT_NUM] = {0};
    double velocity[MAX_JOINT_NUM] = {0};
    double acceleration[MAX_JOINT_NUM] = {0};
    double torque_feedforward[MAX_JOINT_NUM] = {0};
    double torque_external[MAX_JOINT_NUM] = {0};
    double positive_velocity_limit[MAX_JOINT_NUM] = {0};
    double negative_velocity_limit[MAX_JOINT_NUM] = {0};
    double positive_torque_limit[MAX_JOINT_NUM] = {25.0};
    double negative_torque_limit[MAX_JOINT_NUM] = {25.0};
    int chassis_mode = 0;  //底盘控制模式. 0：全向整车控制，1：差速，2：全向电机控制
    // 全向整车控制
    int chassis_whole_mode = 0;
    double chassis_velocity = 0.0;
    double chassis_direction = 0.0;
    // 差速底盘控制
    double chassis_left_velocity = 0.0;
    double chassis_right_velocity = 0.0;
    // 用于全向底盘电机控制的全局变量,左前、右前、左后、右后
    double chassis_motor_velocity[OMNI_MOTOR_NUM] = {0};   //底盘电机控制速度
    double chassis_motor_direction[OMNI_MOTOR_NUM] = {0};  //底盘电机控制方向
    int chassis_estop = 0;
    int chassis_neutral = 0;
    uint64_t chassis_cnt = 0;
};

inline void decodeTwoUint32FromUint64(std::uint64_t v, std::uint32_t& latest, std::uint32_t& reader)
{
    latest = static_cast<std::uint32_t>(v >> 32);
    reader = static_cast<std::uint32_t>(v & 0xFFFFFFFFu);
}

inline std::uint64_t encodeTwoUint32AsUint64(std::uint32_t latest, std::uint32_t reader)
{
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

    bool AppendNewElement(const T& data);
    bool PeekLatest(T* data);
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

    SharedMemory() : shm_ptr_(nullptr)
    {
        shmid_ = shmget(SHM_KEY, sizeof(ShmStruct), IPC_CREAT | IPC_EXCL | 0666);
        bool created = false;
        if (shmid_ < 0)
        {
            if (errno == EEXIST)
            {
                shmid_ = shmget(SHM_KEY, sizeof(ShmStruct), 0666);
                if (shmid_ < 0)
                {
                    std::cerr << "shmget failed, errno=" << errno << " (" << strerror(errno) << ")\n";
                    valid_ = false;
                    return;
                }
            }
            else
            {
                std::cerr << "shmget failed, errno=" << errno << " (" << strerror(errno) << ")\n";
                valid_ = false;
                return;
            }
        }
        else
        {
            created = true;
        }
        void* tmp = shmat(shmid_, NULL, 0);
        if (tmp == (void*)(-1))
        {
            std::cerr << "shmat failed, errno=" << errno << " (" << strerror(errno) << ")\n";
            valid_ = false;
            return;
        }
        shm_ptr_ = reinterpret_cast<ShmStruct*>(tmp);
        if (created)
        {
            // 在共享内存中值初始化构造对象
            new (shm_ptr_) ShmStruct();

            // 设置 latest/reader 初值（避免 0 的歧义，显式使用 0u）
            shm_ptr_->latest_and_reader.store(encodeTwoUint32AsUint64(0u, kInvalidReader), std::memory_order_relaxed);
        }
        valid_ = true;
    }

    ~SharedMemory()
    {
        if (shm_ptr_)
        {
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
                std::cout << "buf.shm_nattch = " << buf.shm_nattch << "."
                          << " Shared memory segment still in use." << std::endl;
            }
        }
    }

    void tryWrapLatestAndReader(std::uint32_t& latest, std::uint32_t& reader);
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

template <typename T, std::uint64_t BufferCapacity, key_t SHM_KEY>
void SharedMemory<T, BufferCapacity, SHM_KEY>::tryWrapLatestAndReader(std::uint32_t& latest, std::uint32_t& reader)
{
    static constexpr std::uint32_t kWrappingRatio = 100;
    static constexpr std::uint32_t kWrappingNumber = kWrappingRatio * BufferCapacity;
    constexpr std::uint32_t kWrapSubtraction = kWrappingNumber - BufferCapacity;
    if (latest > kWrappingNumber)
    {
        if (reader == kInvalidReader)
        {
            latest -= kWrapSubtraction;
        }
        else if (reader > kWrappingNumber)
        {
            latest -= kWrapSubtraction;
            reader -= kWrapSubtraction;
        }
    }
}

template <typename T, std::uint64_t BufferCapacity, key_t SHM_KEY>
bool SharedMemory<T, BufferCapacity, SHM_KEY>::AppendNewElement(const T& element)
{
    std::uint64_t latest_and_reader = shm_ptr_->latest_and_reader.load(std::memory_order_acquire);
    std::uint32_t latest, reader;
    decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
    assert((latest >= reader) || (reader == kInvalidReader));

    // Latest is a valid position, we would write to latest + 1
    const std::uint32_t new_latest = latest + 1;
    const std::uint32_t expected_place = new_latest % BufferCapacity;
    if (reader != kInvalidReader && (new_latest >= reader + BufferCapacity))
    {
        // Can NOT write the given position as someone is reading that
        return false;
    }

    // Now we can write
    QueueElement queue_element;
    queue_element.data = element;
    queue_element.valid = true;
    queue_element.cnt = ++cnt_;
    clock_gettime(CLOCK_REALTIME, &queue_element.ts);

    shm_ptr_->ring_buffer[expected_place] = std::move(queue_element);

    // Perform wrapping
    std::uint32_t wrap_latest = new_latest;
    std::uint32_t wrap_reader = reader;
    tryWrapLatestAndReader(wrap_latest, wrap_reader);

    // Update the index
    std::uint64_t new_latest_and_reader = encodeTwoUint32AsUint64(wrap_latest, wrap_reader);
    while (!shm_ptr_->latest_and_reader.compare_exchange_weak(latest_and_reader, new_latest_and_reader, std::memory_order_acq_rel,
                                                              std::memory_order_relaxed))
    {
        // Update the reader
        decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
        assert((latest >= reader) || (reader == kInvalidReader));

        // Perform wrapping
        wrap_latest = new_latest;
        wrap_reader = reader;
        tryWrapLatestAndReader(wrap_latest, wrap_reader);

        // Encode again
        new_latest_and_reader = encodeTwoUint32AsUint64(wrap_latest, wrap_reader);
    }

    // Done
    return true;
}

template <typename T, std::uint64_t BufferCapacity, key_t SHM_KEY>
bool SharedMemory<T, BufferCapacity, SHM_KEY>::PeekLatest(T* element_output)
{
    bool read_ok = true;
    std::uint32_t latest = 0;
    std::uint32_t reader = 0;
    std::uint64_t latest_and_reader_reserved = 0;
    std::uint64_t latest_and_reader = shm_ptr_->latest_and_reader.load(std::memory_order_acquire);

    // First "lock" a place to read, no wrapping here
    do
    {
        decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
        assert(reader == kInvalidReader);
        latest_and_reader_reserved = encodeTwoUint32AsUint64(latest, latest);
    } while (!shm_ptr_->latest_and_reader.compare_exchange_weak(latest_and_reader, latest_and_reader_reserved, std::memory_order_acq_rel,
                                                                std::memory_order_relaxed));

    // Check the data validity and timeout BEFORE writing to element_output
    const std::uint32_t latest_place = latest % BufferCapacity;
    const auto& elem = shm_ptr_->ring_buffer[latest_place];
    if (!elem.valid)
    {
        read_ok = false;
    }

    if (read_ok)
    {
        int64_t timeout = timeout_ms_.load(std::memory_order_relaxed);
        if (timeout != -1)
        {
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);
            int64_t diff = (now.tv_sec - elem.ts.tv_sec) * 1000LL + (now.tv_nsec - elem.ts.tv_nsec) / 1000000LL;
            if (diff > timeout)
            {
                read_ok = false;
            }
        }
    }

    // Only overwrite output when not timed out and data is valid
    if (element_output != nullptr && read_ok)
    {
        *element_output = elem.data;
        element_output->cnt = elem.cnt;
    }

    // Now release the lock via updating the reader to invalid
    // Also no wrapping here
    std::uint64_t latest_and_reader_released = encodeTwoUint32AsUint64(latest, kInvalidReader);
    while (!shm_ptr_->latest_and_reader.compare_exchange_weak(latest_and_reader, latest_and_reader_released, std::memory_order_acq_rel,
                                                              std::memory_order_relaxed))
    {
        decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
        assert(reader != kInvalidReader);  // It is me
        latest_and_reader_released = encodeTwoUint32AsUint64(latest, kInvalidReader);
    }

    // Done
    return read_ok;
}
