#pragma once

namespace robot_server_lib {

  template <typename T, std::uint64_t BufferCapacity>
  void SingleReadWriteLatestQueue<T, BufferCapacity>::tryWrapLatestAndReader(
      std::uint32_t& latest, std::uint32_t& reader) {
    static constexpr std::uint32_t kWrappingRatio = 107;
    static constexpr std::uint32_t kWrappingNumber =
        kWrappingRatio * BufferCapacity;
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
  bool SingleReadWriteLatestQueue<T, BufferCapacity>::AppendNewElement(
      const T& element) {
    std::uint64_t latest_and_reader =
        latest_and_reader_.load(std::memory_order_acquire);
    std::uint32_t latest, reader;
    decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
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
    std::uint64_t new_latest_and_reader =
        encodeTwoUint32AsUint64(wrap_latest, wrap_reader);
    while (!latest_and_reader_.compare_exchange_weak(
        latest_and_reader, new_latest_and_reader, std::memory_order_acq_rel,
        std::memory_order_relaxed)) {
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

  template <typename T, std::uint64_t BufferCapacity>
  bool SingleReadWriteLatestQueue<T, BufferCapacity>::PeekLatest(
      T* element_output) {
    bool read_ok = true;
    std::uint32_t latest = 0;
    std::uint32_t reader = 0;
    std::uint64_t latest_and_reader_reserved = 0;
    std::uint64_t latest_and_reader =
        latest_and_reader_.load(std::memory_order_acquire);

    // First "lock" a place to read, no wrapping here
    do {
      decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
      assert(reader == kInvalidReader);
      latest_and_reader_reserved = encodeTwoUint32AsUint64(latest, latest);
    } while (!latest_and_reader_.compare_exchange_weak(
        latest_and_reader, latest_and_reader_reserved, std::memory_order_acq_rel,
        std::memory_order_relaxed));

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
    std::uint64_t latest_and_reader_released =
        encodeTwoUint32AsUint64(latest, kInvalidReader);
    while (!latest_and_reader_.compare_exchange_weak(
        latest_and_reader, latest_and_reader_released, std::memory_order_acq_rel,
        std::memory_order_relaxed)) {
      decodeTwoUint32FromUint64(latest_and_reader, latest, reader);
      assert(reader != kInvalidReader);  // It is me
      latest_and_reader_released =
          encodeTwoUint32AsUint64(latest, kInvalidReader);
    }

    // Done
    return read_ok;
  }

  template <typename T, std::int64_t ConcurrentReadersHint>
  bool SingleWriterLatestBuffer<T, ConcurrentReadersHint>::PeekLatest(T* out) {
    std::int64_t current_active = active_.load(std::memory_order_acquire);
    if (current_active < 0) return false;  // Not written even once

    // Loop to reserve the active and read lock
    while (true) {
      bool reload_active = false;
      std::int32_t current_placement = current_active % kArrayCapacity;
      std::uint32_t version_and_ref =
          version_refcount_[current_placement].load(std::memory_order_acquire);
      while (true) {  // Inner loop to increase the reference count
        const std::int64_t version_from_slot = (version_and_ref >> 16) & 0xFFFF;
        const std::int64_t version_from_active = current_active & 0xFFFF;
        if (version_and_ref == kWriter ||
            (version_from_slot != version_from_active)) {
          reload_active = true;
          break;
        }

        // Try update
        const auto increased_refcount = version_and_ref + 1;
        const bool reserved =
            version_refcount_[current_placement].compare_exchange_weak(
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
  bool SingleWriterLatestBuffer<T, ConcurrentReadersHint>::AppendNewElement(
      const T& to_write) {
    const std::int64_t current_active = active_.load(std::memory_order_acquire);
    std::int64_t next_active = current_active + 1;
    while (true) {
      // 不要写入 active_
      // Should NOT write the active, and ONLY I am able to change the active
      // assert(to_write_idx >= 0 && to_write_idx < kArrayCapacity);
      if ((next_active - current_active) % kArrayCapacity == 0) continue;

      // Try reserve: 尝试保留一个槽位
      const std::int64_t next_placement = next_active % kArrayCapacity;
      auto current_ver_ref =
          version_refcount_[next_placement].load(std::memory_order_acquire);
      bool reserved = false;
      if ((current_ver_ref & 0xFFFF) == 0) {
        reserved = version_refcount_[next_placement].compare_exchange_strong(
            current_ver_ref, kWriter, std::memory_order_acq_rel);
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
    version_refcount_[next_placement].store(version_w_released,
                                            std::memory_order_release);

    // Publish the result to active
    assert(next_active != current_active);
    active_.store(next_active, std::memory_order_release);

    // Done
    return true;
  }

} // namespace robot_server_lib
