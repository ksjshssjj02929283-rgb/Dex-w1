/*

Distributed under the MIT License (MIT)

    Copyright (c) 2016 Karthik Iyengar

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in the
Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef NANO_LOG_HEADER_GUARD
#define NANO_LOG_HEADER_GUARD

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>
#include <iosfwd>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace nanolog {

// 前向声明
class NanoLogger;
class NanoLogLine;
struct GuaranteedLogger;
struct NonGuaranteedLogger;

// 日志实例管理器，支持多实例日志
class LogManager {
public:
    static LogManager& instance();

    // 创建或获取命名日志实例
    // file_suffix: 日志文件后缀，默认 "txt"，可设置为 "jsonl" 等
    NanoLogger* get_or_create_logger(const std::string& name,
                                     GuaranteedLogger gl,
                                     const std::string& log_directory,
                                     const std::string& log_file_name,
                                     uint32_t roll_size_mb,
                                     uint32_t max_files,
                                     const std::string& file_suffix = "txt");

    // 获取用于 LOG 宏的默认日志实例
    NanoLogger* get_default_logger();

    // 获取默认原子指针（用于 LOG 宏）
    std::atomic<NanoLogger*>* get_default_atomic_logger();

    // 向指定名称的日志实例写入一行日志
    void write_to_logger(const std::string& name, NanoLogLine&& logline);

private:
    LogManager() = default;
    std::map<std::string, std::unique_ptr<NanoLogger>> loggers_;
    NanoLogger* default_logger_ = nullptr;
    std::atomic<NanoLogger*> atomic_default_logger_{nullptr};
};

enum class LogLevel : uint8_t { DEBUG, INFO, WARN, CRIT };

class NanoLogLine {
public:
    // 设置是否输出时间戳（默认输出）
    static void set_enable_timestamp(bool enable) { s_enable_timestamp = enable; }
    static bool get_enable_timestamp() { return s_enable_timestamp; }

    // 静态成员变量声明
    static bool s_enable_timestamp;

    // 构造函数：enable_timestamp 参数控制该日志行是否输出时间戳（默认继承静态配置）
    NanoLogLine(LogLevel level, char const* file, char const* function, uint32_t line, bool enable_timestamp = true);
    ~NanoLogLine();

    NanoLogLine(NanoLogLine&&) = default;
    NanoLogLine& operator=(NanoLogLine&&) = default;

    void stringify(std::ostream& os);

    NanoLogLine& operator<<(char arg);
    NanoLogLine& operator<<(int32_t arg);
    NanoLogLine& operator<<(uint32_t arg);
    NanoLogLine& operator<<(int64_t arg);
    NanoLogLine& operator<<(uint64_t arg);
    NanoLogLine& operator<<(double arg);
    NanoLogLine& operator<<(std::string const& arg);

    template <size_t N>
    NanoLogLine& operator<<(const char (&arg)[N]) {
        encode(string_literal_t(arg));
        return *this;
    }

    // std::vector<double>
    NanoLogLine& operator<<(const std::vector<double>& vec) {
        (*this) << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            (*this) << vec[i];
            if (i != vec.size() - 1) {
                (*this) << ", ";
            }
        }
        (*this) << "]";
        return *this;
    }

    // std::vector<int>
    NanoLogLine& operator<<(const std::vector<int>& vec) {
        (*this) << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            (*this) << static_cast<int32_t>(vec[i]);
            if (i != vec.size() - 1) {
                (*this) << ", ";
            }
        }
        (*this) << "]";
        return *this;
    }

    template <typename Derived>
    NanoLogLine& operator<<(const Eigen::EigenBase<Derived>& expr) {
        // 无括号，无对齐，元素与行之间都用 ", " 分隔
        static const Eigen::IOFormat kFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
        std::ostringstream oss;
        oss << expr.derived().eval().format(kFmt);
        return (*this) << oss.str();
    }

    template <typename Derived>
    NanoLogLine& operator<<(const Eigen::QuaternionBase<Derived>& q) {
        // 同样去掉括号
        std::ostringstream oss;
        oss << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w();
        return (*this) << oss.str();
    }

    template <typename Arg>
    typename std::enable_if<std::is_same<Arg, char const*>::value, NanoLogLine&>::type operator<<(Arg const& arg) {
        encode(arg);
        return *this;
    }

    template <typename Arg>
    typename std::enable_if<std::is_same<Arg, char*>::value, NanoLogLine&>::type operator<<(Arg const& arg) {
        encode(arg);
        return *this;
    }

    struct string_literal_t {
        explicit string_literal_t(char const* s) : m_s(s) {}
        char const* m_s;
    };

private:
    char* buffer();

    template <typename Arg>
    void encode(Arg arg);

    template <typename Arg>
    void encode(Arg arg, uint8_t type_id);

    void encode(char* arg);
    void encode(char const* arg);
    void encode(string_literal_t arg);
    void encode_c_string(char const* arg, size_t length);
    void resize_buffer_if_needed(size_t additional_bytes);
    void stringify(std::ostream& os, char* start, char const* const end);

private:
    size_t m_bytes_used;
    size_t m_buffer_size;
    std::unique_ptr<char[]> m_heap_buffer;
    char m_stack_buffer[256 - 2 * sizeof(size_t) - sizeof(decltype(m_heap_buffer)) - 9 /* Reserved */];
    bool m_enable_timestamp;  // 该日志行是否输出时间戳
};

struct NanoLog {
    /*
     * Ideally this should have been operator+=
     * Could not get that to compile, so here we are...
     */
    bool operator==(NanoLogLine&);
};

void set_log_level(LogLevel level);

bool is_logged(LogLevel level);

/*
 * Non guaranteed logging. Uses a ring buffer to hold log lines.
 * When the ring gets full, the previous log line in the slot will be dropped.
 * Does not block producer even if the ring buffer is full.
 * ring_buffer_size_mb - LogLines are pushed into a mpsc ring buffer whose size
 * is determined by this parameter. Since each LogLine is 256 bytes,
 * ring_buffer_size = ring_buffer_size_mb * 1024 * 1024 / 256
 */
struct NonGuaranteedLogger {
    NonGuaranteedLogger(uint32_t ring_buffer_size_mb_) : ring_buffer_size_mb(ring_buffer_size_mb_) {}
    uint32_t ring_buffer_size_mb;
};

/*
 * Provides a guarantee log lines will not be dropped.
 */
struct GuaranteedLogger {};

/*
 * Ensure initialize() is called prior to any log statements.
 * log_directory - where to create the logs. For example - "/tmp/"
 * log_file_name - root of the file name. For example - "nanolog"
 * This will create rolling log files of the form (max_files = K) -
 * /tmp/nanolog.txt
 * /tmp/nanolog1.txt
 * /tmp/nanolog2.txt
 * ...
 * /tmp/nanolog(K-1).txt
 * The newest is nanolog.txt, the oldest is nanolog(K-1).txt.
 * Rotation happens when nanolog.txt exceeds roll size.
 */
void initialize(GuaranteedLogger gl,
                std::string const& log_directory,
                std::string const& log_file_name,
                uint32_t log_file_roll_size_mb);
void initialize(NonGuaranteedLogger ngl,
                std::string const& log_directory,
                std::string const& log_file_name,
                uint32_t log_file_roll_size_mb);

// New overloads with fixed rolling file count
void initialize(GuaranteedLogger gl,
                std::string const& log_directory,
                std::string const& log_file_name,
                uint32_t log_file_roll_size_mb,
                uint32_t max_files);

void initialize(NonGuaranteedLogger ngl,
                std::string const& log_directory,
                std::string const& log_file_name,
                uint32_t log_file_roll_size_mb,
                uint32_t max_files);

}  // namespace nanolog

#define NANO_LOG(LEVEL) nanolog::NanoLog() == nanolog::NanoLogLine(LEVEL, __FILE__, __func__, __LINE__)
#define LOG_DEBUG nanolog::is_logged(nanolog::LogLevel::DEBUG) && NANO_LOG(nanolog::LogLevel::DEBUG)
#define LOG_INFO nanolog::is_logged(nanolog::LogLevel::INFO) && NANO_LOG(nanolog::LogLevel::INFO)
#define LOG_WARN nanolog::is_logged(nanolog::LogLevel::WARN) && NANO_LOG(nanolog::LogLevel::WARN)
#define LOG_CRIT nanolog::is_logged(nanolog::LogLevel::CRIT) && NANO_LOG(nanolog::LogLevel::CRIT)

#endif /* NANO_LOG_HEADER_GUARD */
