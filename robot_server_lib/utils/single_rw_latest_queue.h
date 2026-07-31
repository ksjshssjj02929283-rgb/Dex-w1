//
// Created by Wei Gao on 2023/2/14.
//

#pragma once

#include <array>
#include <atomic>
#include <cassert>

#include "encoding.h"

namespace robot_server_lib {

template <typename T, std::uint64_t BufferCapacity = 8>
class SingleReadWriteLatestQueue {
public:
    SingleReadWriteLatestQueue();
    ~SingleReadWriteLatestQueue() = default;

    // Append a new element into the queue as the latest one.
    bool AppendNewElement(const T& element);
    // Read the latest element and store it into element_out. This method
    // return false only if no element has been appended.
    bool PeekLatest(T* element_output);
    // Reset the queue to initial state.
    void Reset();

private:
    // The element stored in the queue
    struct QueueElement {
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
    void tryWrapLatestAndReader(std::uint32_t& latest, std::uint32_t& reader);
};

// SingleWriterLatestBuffer 是一个线程安全的缓冲区，允许一个写入者和多个读取者
template <typename T, std::int64_t ConcurrentReadersHint = 3>
class SingleWriterLatestBuffer {
public:
    SingleWriterLatestBuffer() = default;
    ~SingleWriterLatestBuffer() = default;
    // 读取最新数据。它会读取 active_
    // 指向的槽位，并增加该槽位的引用计数，确保数据在读取期间不会被覆盖。
    bool PeekLatest(T* output);
    // 写入新数据。它会寻找一个没有读者正在读取的槽位（version_refcount_ == 0），写入数据后更新
    // active_ 指向新数据。
    bool AppendNewElement(const T& to_write);
    // 重置缓冲区
    void Reset();

private:
    static constexpr std::int64_t kArrayCapacity = ConcurrentReadersHint + 1;
    static constexpr std::uint32_t kWriter = 0xFFFFFFFFu;
    std::array<T, kArrayCapacity> elements_{};
    std::array<std::atomic_uint32_t, kArrayCapacity> version_refcount_{0};
    std::atomic_int64_t active_{-1};
};
template <typename T, std::uint64_t BufferCapacity>
SingleReadWriteLatestQueue<T, BufferCapacity>::SingleReadWriteLatestQueue() {
    static_assert(BufferCapacity >= 2);
}

template <typename T, std::uint64_t BufferCapacity>
void SingleReadWriteLatestQueue<T, BufferCapacity>::tryWrapLatestAndReader(std::uint32_t& latest,
                                                                           std::uint32_t& reader) {
    static constexpr std::uint32_t kWrappingRatio = 107;
    static constexpr std::uint32_t kWrappingNumber = kWrappingRatio * BufferCapacity;
    constexpr std::uint32_t kWrapSubtraction = kWrappingNumber - BufferCapacity;
    if (latest > kWrappingNumber) {
        if (reader == kInvalidReader) {
            latest -= kWrapSubtraction;
        } else if (reader > kWrappingNumber) {
            latest -= kWrapSubtraction;
            reader -= kWrapSubtraction;
        }
    }
}

template <typename T, std::uint64_t BufferCapacity>
bool SingleReadWriteLatestQueue<T, BufferCapacity>::AppendNewElement(const T& element) {
    std::uint64_t latest_and_reader = latest_and_reader_.load(std::memory_order_acquire);
    std::uint32_t latest, reader;
    utils::decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
    assert((latest >= reader) || (reader == kInvalidReader));

    // Latest is a valid position, we would write to latest + 1
    const std::uint32_t new_latest = latest + 1;
    const std::uint32_t expected_place = new_latest % BufferCapacity;
    if (reader != kInvalidReader && (new_latest >= reader + BufferCapacity)) {
        // Can NOT write the given position as someone is reading that
        return false;
    }

    // Now we can write
    QueueElement queue_element;
    queue_element.data = element;
    queue_element.valid = true;
    ring_buffer_[expected_place] = std::move(queue_element);

    // Perform wrapping
    std::uint32_t wrap_latest = new_latest;
    std::uint32_t wrap_reader = reader;
    tryWrapLatestAndReader(wrap_latest, wrap_reader);

    // Update the index
    std::uint64_t new_latest_and_reader = utils::encodeTwoUint32AsUint64(wrap_latest, wrap_reader);
    while (!latest_and_reader_.compare_exchange_weak(latest_and_reader, new_latest_and_reader,
                                                     std::memory_order_acq_rel, std::memory_order_relaxed)) {
        // Update the reader
        utils::decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
        assert((latest >= reader) || (reader == kInvalidReader));

        // Perform wrapping
        wrap_latest = new_latest;
        wrap_reader = reader;
        tryWrapLatestAndReader(wrap_latest, wrap_reader);

        // Encode again
        new_latest_and_reader = utils::encodeTwoUint32AsUint64(wrap_latest, wrap_reader);
    }

    // Done
    return true;
}

template <typename T, std::uint64_t BufferCapacity>
bool SingleReadWriteLatestQueue<T, BufferCapacity>::PeekLatest(T* element_output) {
    bool read_ok = true;
    std::uint32_t latest = 0;
    std::uint32_t reader = 0;
    std::uint64_t latest_and_reader_reserved = 0;
    std::uint64_t latest_and_reader = latest_and_reader_.load(std::memory_order_acquire);

    // First "lock" a place to read, no wrapping here
    do {
        utils::decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
        assert(reader == kInvalidReader);
        latest_and_reader_reserved = utils::encodeTwoUint32AsUint64(latest, latest);
    } while (!latest_and_reader_.compare_exchange_weak(latest_and_reader, latest_and_reader_reserved,
                                                       std::memory_order_acq_rel, std::memory_order_relaxed));

    // Check the data validity and read it
    const std::uint32_t latest_place = latest % BufferCapacity;
    if (!ring_buffer_[latest_place].valid) {
        read_ok = false;
    }

    // Now the data is valid
    if (element_output != nullptr && read_ok) {
        *element_output = ring_buffer_[latest_place].data;
    }

    // Now release the lock via updating the reader to invalid
    // Also no wrapping here
    std::uint64_t latest_and_reader_released = utils::encodeTwoUint32AsUint64(latest, kInvalidReader);
    while (!latest_and_reader_.compare_exchange_weak(latest_and_reader, latest_and_reader_released,
                                                     std::memory_order_acq_rel, std::memory_order_relaxed)) {
        utils::decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
        assert(reader != kInvalidReader);  // It is me
        latest_and_reader_released = utils::encodeTwoUint32AsUint64(latest, kInvalidReader);
    }

    // Done
    return read_ok;
}

template <typename T, std::uint64_t BufferCapacity>
void SingleReadWriteLatestQueue<T, BufferCapacity>::Reset() {
    // 清空环形缓冲区内容与有效位
    for (auto& slot : ring_buffer_) {
        slot.data = T{};
        slot.valid = false;
    }
    // 重置索引：latest=0, reader=kInvalidReader
    latest_and_reader_.store(utils::encodeTwoUint32AsUint64(0u, kInvalidReader), std::memory_order_release);
}

template <typename T, std::int64_t ConcurrentReadersHint>
bool SingleWriterLatestBuffer<T, ConcurrentReadersHint>::PeekLatest(T* out) {
    std::int64_t current_active = active_.load(std::memory_order_acquire);
    if (current_active < 0) return false;  // Not written even once

    // Loop to reserve the active and read lock
    while (true) {
        bool reload_active = false;
        std::int32_t current_placement = current_active % kArrayCapacity;
        std::uint32_t version_and_ref = version_refcount_[current_placement].load(std::memory_order_acquire);
        while (true) {  // Inner loop to increase the reference count
            const std::int64_t version_from_slot = (version_and_ref >> 16) & 0xFFFF;
            const std::int64_t version_from_active = current_active & 0xFFFF;
            if (version_and_ref == kWriter || (version_from_slot != version_from_active)) {
                reload_active = true;
                break;
            }

            // Try update
            const auto increased_refcount = version_and_ref + 1;
            const bool reserved = version_refcount_[current_placement].compare_exchange_weak(
                version_and_ref, increased_refcount, std::memory_order_acq_rel);
            if (reserved) break;
        }

        // End of inner loop, check reason
        if (!reload_active) break;

        // Load it again
        current_active = active_.load(std::memory_order_acquire);
    }

    // Should be reserved, perform reading
    const std::int64_t current_placement = current_active % kArrayCapacity;
    if (out) *out = elements_[current_placement];

    // Release the read-lock and return
    version_refcount_[current_placement].fetch_sub(1, std::memory_order_release);
    return true;
}

template <typename T, std::int64_t ConcurrentReadersHint>
bool SingleWriterLatestBuffer<T, ConcurrentReadersHint>::AppendNewElement(const T& to_write) {
    const std::int64_t current_active = active_.load(std::memory_order_acquire);
    std::int64_t next_active = current_active + 1;
    while (true) {
        // 不要写入 active_
        // Should NOT write the active, and ONLY I am able to change the active
        // assert(to_write_idx >= 0 && to_write_idx < kArrayCapacity);
        if ((next_active - current_active) % kArrayCapacity == 0) continue;

        // Try reserve: 尝试保留一个槽位
        const std::int64_t next_placement = next_active % kArrayCapacity;
        auto current_ver_ref = version_refcount_[next_placement].load(std::memory_order_acquire);
        bool reserved = false;
        if ((current_ver_ref & 0xFFFF) == 0) {
            reserved = version_refcount_[next_placement].compare_exchange_strong(current_ver_ref, kWriter,
                                                                                 std::memory_order_acq_rel);
        }

        if (reserved) break;

        // Not succeed
        next_active += 1;
    }

    // Write it
    const std::int64_t next_placement = next_active % kArrayCapacity;
    elements_[next_placement] = to_write;

    // Release the write lock to indicate writing finish
    assert(version_refcount_[next_placement].load() == kWriter);
    std::uint32_t version_w_released = (next_active & 0xFFFF) << 16;
    version_refcount_[next_placement].store(version_w_released, std::memory_order_release);

    // Publish the result to active
    assert(next_active != current_active);
    active_.store(next_active, std::memory_order_release);

    // Done
    return true;
}

template <typename T, std::int64_t ConcurrentReadersHint>
void SingleWriterLatestBuffer<T, ConcurrentReadersHint>::Reset() {
    // 清空元素内容
    for (auto& e : elements_) e = T{};

    // 清空版本与引用计数
    for (auto& vr : version_refcount_) vr.store(0u, std::memory_order_relaxed);

    // 重置 active
    active_.store(-1, std::memory_order_release);
}

}  // namespace robot_server_lib