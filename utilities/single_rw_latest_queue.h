//
// Created by Wei Gao on 2023/2/14.
//

#pragma once

#include <array>
#include <atomic>
#include <cassert>

#include "encoding.h"

namespace robot_server_lib
{

template <typename T, std::uint64_t BufferCapacity = 8>
class SingleReadWriteLatestQueue
{
   public:
    SingleReadWriteLatestQueue();
    ~SingleReadWriteLatestQueue() = default;

    // Append a new element into the queue as the latest one.
    bool AppendNewElement(const T &element);
    // Read the latest element and store it into element_out. This method
    // return false only if no element has been appended.
    bool PeekLatest(T *element_output);

   private:
    // The element stored in the queue
    struct QueueElement
    {
        T data;
        bool valid{false};  // Handle the case if no element has arrived yet
    };

    // The actual buffer
    std::array<QueueElement, BufferCapacity> ring_buffer_;

    // 索引方案说明：
    // latest 表示最新元素的位置，(latest % capacity) 可用于获取该元素。
    //     latest 初始化为零，可以无限递增，因此需要包装处理。
    // reader 表示当前有一个线程正在读取的元素位置。默认值 kInvalidReader 表示没有线程在读取。
    //     reader 初始化为该默认值，读取时会设置为实际正在读取的元素位置。
    // latest 和 reader 组合成 64 位成员 latest_and_reader_，
    //     以实现原子操作。AppendNewElement 会将 latest 更新为 latest + 1，
    //     但不会修改 reader，通过 reader 确保 latest + 1 的内存位置可写。
    // Peek 会更新 reader 以“锁定”一个内存位置用于读取，读取完成后会“释放”该内存位置。
    // The indexing scheme:
    // latest is the place of the newest element and (latest % capacity) can be
    //     used to retrieve that element. latest is initialized to zero and can
    //     go to infinity, thus we need wrapping to handle that
    // reader indicates the given element are being read by ONE thread. There is
    //     a default kInvalidReader implies no one is reading. reader is
    //     initialized to that value and set to the actual element being read
    // latest and reader are combined into a 64 bit @member latest_and_reader_
    //     to achieve atomic operation. AppendNewElement would update latest to
    //     latest + 1 without mutating reader, through it use reader to ensure
    //     the memory location for latest + 1 can be written. Peek will update
    //     reader to "lock" a memory location for reading, upon finish the memory
    //     location will be "released".
    static constexpr std::uint32_t kInvalidReader = 0xFFFFFFFFu;
    std::atomic_uint64_t latest_and_reader_{kInvalidReader};
    // Note: latest_and_reader_ should be init to encode(0, kInvalidReader)

    // For wrapping
    void tryWrapLatestAndReader(std::uint32_t &latest, std::uint32_t &reader);
};

// SingleWriterLatestBuffer 是一个线程安全的缓冲区，允许一个写入者和多个读取者
template <typename T, std::int64_t ConcurrentReadersHint = 3>
class SingleWriterLatestBuffer
{
   public:
    SingleWriterLatestBuffer() = default;
    ~SingleWriterLatestBuffer() = default;
    // 读取最新数据。它会读取 active_
    // 指向的槽位，并增加该槽位的引用计数，确保数据在读取期间不会被覆盖。
    bool PeekLatest(T *output);
    // 写入新数据。它会寻找一个没有读者正在读取的槽位（version_refcount_ == 0），写入数据后更新
    // active_ 指向新数据。
    bool AppendNewElement(const T &to_write);

   private:
    static constexpr std::int64_t kArrayCapacity = ConcurrentReadersHint + 1;
    static constexpr std::uint32_t kWriter = 0xFFFFFFFFu;
    std::array<T, kArrayCapacity> elements_{};
    std::array<std::atomic_uint32_t, kArrayCapacity> version_refcount_{0};
    std::atomic_int64_t active_{-1};
};
template <typename T, std::uint64_t BufferCapacity>
SingleReadWriteLatestQueue<T, BufferCapacity>::SingleReadWriteLatestQueue()
{
    static_assert(BufferCapacity >= 2);
}

}  // namespace robot_server_lib

#include "single_rw_latest_queue.hpp"