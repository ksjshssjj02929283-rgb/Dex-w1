#pragma once

#include <cassert>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <unordered_map>

namespace robot_server_lib {

/// 单写者-单读者 key-value 消息缓冲, 按 key 精确存取
/// 写者线程调用 Put, 读者线程调用 TakeNonBlocking / TakeBlocking
template <typename T>
class SingleReaderMessageBufferMap {
public:
    SingleReaderMessageBufferMap() = default;
    ~SingleReaderMessageBufferMap() = default;

    /// 写入: 按 key 存入 message, key 重复时返回 false
    bool Put(std::int64_t key, T message) {
        bool notify_cv = false;
        {
            std::lock_guard<std::mutex> guard(mutex_);
            auto result = message_map_.insert(std::make_pair(key, std::move(message)));
            if (!result.second) return false;

            if (waiting_reader_.is_valid && key == waiting_reader_.request_key) {
                *waiting_reader_.request_arrived = true;
                if (waiting_reader_.result_destination != nullptr) {
                    *waiting_reader_.result_destination = result.first->second;
                }
                notify_cv = true;
            }
        }
        if (notify_cv) {
            waiting_reader_.cv.notify_one();
        }
        return true;
    }

    /// 非阻塞读取并删除: key 不存在时立即返回 false
    bool TakeNonBlocking(std::int64_t key, T* message) {
        std::lock_guard<std::mutex> guard(mutex_);
        return getMessageInternal(key, message, true);
    }

    /// 阻塞读取并删除: 等待直到 key 出现或超时
    bool TakeBlocking(std::int64_t key, T* message, int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (getMessageInternal(key, message, true)) return true;

        assert(!waiting_reader_.is_valid);
        bool request_arrived = false;
        waiting_reader_.is_valid = true;
        waiting_reader_.request_key = key;
        waiting_reader_.result_destination = message;
        waiting_reader_.request_arrived = &request_arrived;

        auto block_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (!request_arrived) {
            if (waiting_reader_.cv.wait_until(lock, block_until) == std::cv_status::timeout) {
                break;
            }
        }

        waiting_reader_.is_valid = false;

        if (request_arrived) {
            getMessageInternal(key, nullptr, true);
            return true;
        }
        return getMessageInternal(key, message, true);
    }

    /// 非阻塞读取不删除: key 不存在时返回 false
    bool Peek(std::int64_t key, T* message) {
        std::lock_guard<std::mutex> guard(mutex_);
        return getMessageInternal(key, message, false);
    }

    /// 清理: 按条件删除 key (仅读者线程调用)
    std::size_t RemoveKeysByFunctor(const std::function<bool(std::int64_t)>& should_remove) {
        std::lock_guard<std::mutex> guard(mutex_);
        std::size_t count = 0;
        for (auto it = message_map_.begin(); it != message_map_.end();) {
            if (should_remove(it->first)) {
                it = message_map_.erase(it);
                ++count;
            } else {
                ++it;
            }
        }
        return count;
    }

private:
    bool getMessageInternal(std::int64_t key, T* message, bool remove) {
        auto iter = message_map_.find(key);
        if (iter == message_map_.end()) return false;
        if (message != nullptr) *message = iter->second;
        if (remove) message_map_.erase(iter);
        return true;
    }

    std::mutex mutex_;
    std::unordered_map<std::int64_t, T> message_map_;

    struct {
        bool is_valid{false};
        std::condition_variable cv{};
        std::int64_t request_key{};
        T* result_destination{nullptr};
        bool* request_arrived{nullptr};
    } waiting_reader_;
};

}  // namespace robot_server_lib
