//
// Created by Wei Gao on 2/13/2023.
//

#pragma once

#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <vector>

namespace robot_server_lib {

template <typename T>
class BoundedQueue {
private:
    // The buffer for the data, the size is not mutable after construction
    // T should support default construction && copy, it is usually POD type
    std::vector<T> ring_buffer_;

    // The index scheme is very similar to the one in SingleRWQueue, except that
    // we need a @member commit_ field to handle the multiple write. end_ is used
    // to reserve the buffer place while commit_ is used to flag the finish of
    // writing.
    // The valid (committed) range is [begin, committed)
    // Everything before (not include) @member reserved_end_ will
    // be committed, in other words, [begin, reserved_end) will eventually
    // be committed
    std::atomic_uint64_t begin_{0};
    std::atomic_uint64_t reserve_end_{0};
    std::atomic_uint64_t commit_end_{0};

public:
    explicit BoundedQueue(std::uint64_t capacity);
    ~BoundedQueue() = default;
    // No-copy/assign/move, as usual in concurrent code
    BoundedQueue(const BoundedQueue&) = delete;
    BoundedQueue(BoundedQueue&&) = delete;

    // Actual interface
    bool EnQueue(const T& element);
    bool DeQueue(T* element_output = nullptr);
    std::uint64_t ElementSize();
    std::uint64_t capacity() const;
};

template <typename T>
BoundedQueue<T>::BoundedQueue(std::uint64_t capacity) : ring_buffer_(capacity) {}

template <typename T>
bool BoundedQueue<T>::EnQueue(const T& element) {
    const auto buffer_capacity = capacity();
    std::uint64_t current_begin = 0;
    std::uint64_t new_reserve_end = 0;
    std::uint64_t planned_placement = 0;
    std::uint64_t current_size_include_reserved = 0;
    std::uint64_t old_reserve_end = reserve_end_.load(std::memory_order_acquire);

    // First loop reserve the placement location at new_end
    do {
        // Planned placement location
        new_reserve_end = old_reserve_end + 1;
        planned_placement = old_reserve_end % buffer_capacity;

        // Check if it there is more place to write
        current_begin = begin_.load(std::memory_order_acquire);

        // As detailed below, this might not represent the actual size
        // as current_begin might be larger than old_end
        current_size_include_reserved = old_reserve_end - current_begin;

        // This assert may fail despite the operation is correct
        // After the last read of end_ to get old_end, there might be
        // many push/pop on other reader/writer threads which update
        // both end_ and begin_, which leads to begin_ much larger than
        // old_end
        // assert(current_size_include_reserved <= buffer_capacity);

        // If check passed, then ok to write in placement
        if ((old_reserve_end >= current_begin) && (current_size_include_reserved == buffer_capacity)) {
            return false;
        }

        // Try update the end to reserve place
    } while (!reserve_end_.compare_exchange_weak(old_reserve_end, new_reserve_end, std::memory_order_acq_rel,
                                                 std::memory_order_relaxed));

    // Place reserve ok
    ring_buffer_[planned_placement] = element;

    // Copy done, try update commit to new end
    // Only perform update when old_commit is old_end
    std::uint64_t expected_old_commit = old_reserve_end;
    while (!commit_end_.compare_exchange_weak(expected_old_commit, new_reserve_end, std::memory_order_acq_rel,
                                              std::memory_order_relaxed)) {
        assert(expected_old_commit <= old_reserve_end);  // This is weak
        expected_old_commit = old_reserve_end;
    }

    // Done
    return true;
}

template <typename T>
bool BoundedQueue<T>::DeQueue(T* element_output) {
    const std::uint64_t buffer_capacity = capacity();
    std::uint64_t new_begin = 0;
    std::uint64_t old_begin_place = 0;
    std::uint64_t current_commit = 0;
    std::uint64_t old_begin = begin_.load(std::memory_order_acquire);

    // The loop
    do {
        // This must be a valid place that we can get value
        // if the element size is not empty
        current_commit = commit_end_.load(std::memory_order_acquire);
        assert(old_begin <= current_commit);

        // Old implementation
        // current_size_commit_only = current_commit - old_begin;
        // assert(current_size_commit_only <= buffer_capacity);
        // Thus, we use this condition:
        if (current_commit == old_begin) {
            return false;
        }

        // Now we can retrieve the element in old_place
        old_begin_place = old_begin % buffer_capacity;
        if (element_output != nullptr) {
            *element_output = ring_buffer_[old_begin_place];
        }

        // Try update the begin
        new_begin = old_begin + 1;
    } while (!begin_.compare_exchange_weak(old_begin, new_begin, std::memory_order_acq_rel, std::memory_order_relaxed));

    // Done
    return true;
}

template <typename T>
std::uint64_t BoundedQueue<T>::ElementSize() {
    std::uint64_t begin = begin_.load();
    std::uint64_t commit_end = commit_end_.load();
    assert(commit_end >= begin);
    return commit_end - begin;
}

template <typename T>
std::uint64_t BoundedQueue<T>::capacity() const {
    return ring_buffer_.size();
}

}  // namespace robot_server_lib