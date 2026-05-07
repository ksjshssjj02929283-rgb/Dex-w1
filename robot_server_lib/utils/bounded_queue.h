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

}  // namespace robot_server_lib

#include "bounded_queue.hpp"