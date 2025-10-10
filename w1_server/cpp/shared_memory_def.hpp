#pragma once

template <typename T, std::uint64_t BufferCapacity, key_t SHM_KEY>
void SharedMemory<T, BufferCapacity, SHM_KEY>::tryWrapLatestAndReader(std::uint32_t &latest, std::uint32_t &reader)
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
bool SharedMemory<T, BufferCapacity, SHM_KEY>::AppendNewElement(const T &element)
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
    while (!shm_ptr_->latest_and_reader.compare_exchange_weak(latest_and_reader, new_latest_and_reader, std::memory_order_acq_rel, std::memory_order_relaxed))
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
bool SharedMemory<T, BufferCapacity, SHM_KEY>::PeekLatest(T *element_output)
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

    // Check the data validity and read it
    const std::uint32_t latest_place = latest % BufferCapacity;
    if (!shm_ptr_->ring_buffer[latest_place].valid)
    {
        read_ok = false;
    }

    // Now the data is valid
    if (element_output != nullptr && read_ok)
    {
        *element_output = shm_ptr_->ring_buffer[latest_place].data;
        element_output->cnt = shm_ptr_->ring_buffer[latest_place].cnt;
        auto data_ts = shm_ptr_->ring_buffer[latest_place].ts;
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        int64_t timeout = timeout_ms_.load(std::memory_order_relaxed);
        // -1时不检测超时
        if (timeout != -1)
        {
            int64_t diff = (now.tv_sec - data_ts.tv_sec) * 1000LL + (now.tv_nsec - data_ts.tv_nsec) / 1000000LL;
            if (diff > timeout)
            {
                // std::cout << "[SharedMemory] data read timeout: " << diff << " ms" << std::endl;
                read_ok = false;
            }
        }
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
    
