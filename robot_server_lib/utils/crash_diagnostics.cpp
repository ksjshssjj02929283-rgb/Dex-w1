#include "crash_diagnostics.h"

#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/utsname.h>
#include <ucontext.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace robot_server_lib::crash_diagnostics {
namespace {
int crash_fd = -1;  // Open before handlers/threads, never closed during process lifetime.
std::atomic<bool> enabled{false};
constexpr int fatal_signals[] = {SIGSEGV, SIGBUS, SIGABRT, SIGILL, SIGFPE};

// No allocation, stdio, locks, symbol lookup, or logging library in this path.
struct Record {
    char data[1024];
    size_t size = 0;
    void Text(const char* text) noexcept {
        while (*text && size < sizeof(data)) data[size++] = *text++;
    }
    void Number(uint64_t value, unsigned base = 10) noexcept {
        char digits[32];
        size_t count = 0;
        do {
            digits[count++] = "0123456789abcdef"[value % base];
            value /= base;
        } while (value);
        while (count && size < sizeof(data)) data[size++] = digits[--count];
    }
    void Signed(int value) noexcept {
        if (value < 0) Text("-");
        Number(value < 0 ? -static_cast<int64_t>(value) : value);
    }
    void Hex(const char* key, uint64_t value) noexcept {
        Text(key);
        Text("0x");
        Number(value, 16);
    }
};

void WriteRecord(const char* data, size_t size) noexcept {
    // One bounded attempt: do not spin if storage is failing. O_APPEND keeps
    // records independent of the shared file offset. No durability guarantee.
    const auto ignored = write(crash_fd, data, size);
    (void)ignored;
}

void HandleFatalSignal(int signal_number, siginfo_t* info, void* context) noexcept {
    const int saved_errno = errno;
    Record record;
    record.Text("CRASH realtime_sec=");
    timespec now{};
    clock_gettime(CLOCK_REALTIME, &now);
    record.Number(static_cast<uint64_t>(now.tv_sec));
    record.Text(" realtime_nsec=");
    record.Number(static_cast<uint64_t>(now.tv_nsec));
    record.Text(" pid=");
    record.Number(static_cast<uint64_t>(getpid()));
    record.Text(" tid=");
    // Linux-only direct syscall wrapper; no thread-local initialization.
    record.Number(static_cast<uint64_t>(syscall(SYS_gettid)));
    record.Text(" signal=");
    record.Signed(signal_number);
    record.Text(" si_code=");
    record.Signed(info ? info->si_code : 0);
    if (info && info->si_code > 0 &&
        (signal_number == SIGSEGV || signal_number == SIGBUS ||
         signal_number == SIGILL || signal_number == SIGFPE)) {
        record.Hex(" fault_addr=", reinterpret_cast<uintptr_t>(info->si_addr));
    }
    if (context) {
        const auto* uc = static_cast<const ucontext_t*>(context);
#if defined(__aarch64__)
        record.Hex(" pc=", uc->uc_mcontext.pc);
        record.Hex(" sp=", uc->uc_mcontext.sp);
        record.Hex(" fp=", uc->uc_mcontext.regs[29]);
        record.Hex(" lr=", uc->uc_mcontext.regs[30]);
#elif defined(__x86_64__)
        record.Hex(" pc=", uc->uc_mcontext.gregs[REG_RIP]);
        record.Hex(" sp=", uc->uc_mcontext.gregs[REG_RSP]);
        record.Hex(" fp=", uc->uc_mcontext.gregs[REG_RBP]);
#endif
    }
    record.Hex(" handler_stack_addr=", reinterpret_cast<uintptr_t>(&record));
    record.Text("\n");
    WriteRecord(record.data, record.size);

    // Preserve signal termination and core dumping. The signal is blocked
    // during this handler; raise queues it on THIS thread, and returning
    // restores the original context/mask before default delivery. Never use
    // _exit on the normal path (it would suppress the core).
    struct sigaction action{};
    action.sa_handler = SIG_DFL;
    sigemptyset(&action.sa_mask);
    if (sigaction(signal_number, &action, nullptr) != 0 || raise(signal_number) != 0) {
        _exit(128 + signal_number);
    }
    errno = saved_errno;
}

void Check(bool success, const char* operation) {
    if (!success) throw std::system_error(errno, std::generic_category(), operation);
}

struct AlternateStack {
    void* mapping = MAP_FAILED;
    size_t mapping_size = 0;
    stack_t previous{};
    void Install() {
        if (mapping != MAP_FAILED) return;
        const long page_size = sysconf(_SC_PAGESIZE);
        if (page_size <= 0) throw std::runtime_error("cannot determine page size");
        const size_t page = static_cast<size_t>(page_size);
        const size_t usable = ((128 * 1024 + page - 1) / page) * page;
        mapping_size = usable + 2 * page;
        mapping = mmap(nullptr, mapping_size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        Check(mapping != MAP_FAILED, "mmap alternate signal stack");
        auto* start = static_cast<char*>(mapping) + page;
        if (mprotect(start, usable, PROT_READ | PROT_WRITE) != 0) {
            const int error = errno;
            munmap(mapping, mapping_size);
            mapping = MAP_FAILED;
            throw std::system_error(error, std::generic_category(), "mprotect alternate stack");
        }
        // Touch pages before any real-time scheduling is enabled.
        std::memset(start, 0, usable);
        stack_t stack{};
        stack.ss_sp = start;
        stack.ss_size = usable;
        if (sigaltstack(&stack, &previous) != 0) {
            const int error = errno;
            munmap(mapping, mapping_size);
            mapping = MAP_FAILED;
            throw std::system_error(error, std::generic_category(), "sigaltstack");
        }
    }
    ~AlternateStack() {
        if (mapping != MAP_FAILED && sigaltstack(&previous, nullptr) == 0) {
            munmap(mapping, mapping_size);
        }
    }
};

std::string ReadFile(const char* path) {
    std::ifstream input(path);
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}
}  // namespace

void RegisterCurrentThread(const char* name) {
    if (!enabled.load(std::memory_order_acquire)) return;
    thread_local AlternateStack alternate_stack;
    alternate_stack.Install();
    const int error = pthread_setname_np(pthread_self(), name);
    if (error != 0) throw std::system_error(error, std::generic_category(), "pthread_setname_np");
    Record record;
    record.Text("THREAD tid=");
    record.Number(static_cast<uint64_t>(syscall(SYS_gettid)));
    record.Text(" name=");
    record.Text(name);
    record.Hex(" alt_mapping=", reinterpret_cast<uintptr_t>(alternate_stack.mapping));
    record.Text(" alt_mapping_size=");
    record.Number(alternate_stack.mapping_size);
    record.Text("\n");
    WriteRecord(record.data, record.size);
}

void RecordProcessMaps() {
    if (!enabled.load(std::memory_order_acquire)) return;
    const std::string maps = "MAPS_BEGIN\n" + ReadFile("/proc/self/maps") + "MAPS_END\n";
    WriteRecord(maps.data(), maps.size());
}

void Initialize(const std::string& directory, const std::string& version) {
    if (enabled.load()) throw std::logic_error("crash diagnostics already initialized");
    std::filesystem::create_directories(directory);
    timespec now{};
    Check(clock_gettime(CLOCK_REALTIME, &now) == 0, "clock_gettime");
    const auto file = std::filesystem::path(directory) /
        ("w1_crash_" + std::to_string(now.tv_sec) + "_" + std::to_string(getpid()) + "_" +
         std::to_string(now.tv_nsec) + ".log");
    crash_fd = open(file.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_APPEND | O_CLOEXEC | O_NOFOLLOW, 0600);
    Check(crash_fd >= 0, "open crash log");

    // File capabilities used by run_w1.sh may clear dumpability. Opt this
    // process in, without weakening global fs.suid_dumpable policy.
    const int dumpable_before = prctl(PR_GET_DUMPABLE);
    Check(prctl(PR_SET_DUMPABLE, 1) == 0, "PR_SET_DUMPABLE");
    rlimit limit{};
    Check(getrlimit(RLIMIT_CORE, &limit) == 0, "getrlimit core");
    limit.rlim_cur = limit.rlim_max;
    Check(setrlimit(RLIMIT_CORE, &limit) == 0, "setrlimit core");

    utsname system{};
    uname(&system);
    const std::string pattern = ReadFile("/proc/sys/kernel/core_pattern");
    std::ostringstream metadata;
    metadata << "START version=" << version << " pid=" << getpid()
             << " kernel=" << system.release << " arch=" << system.machine
             << " dumpable_before=" << dumpable_before << " dumpable=" << prctl(PR_GET_DUMPABLE)
             << " core_soft=" << limit.rlim_cur << " core_hard=" << limit.rlim_max
             << "\ncore_pattern=" << pattern
             << "executable=" << std::filesystem::read_symlink("/proc/self/exe").string()
             << "\ncwd=" << std::filesystem::current_path().string() << "\n";
    const std::string text = metadata.str();
    Check(write(crash_fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()), "write crash metadata");

    enabled.store(true, std::memory_order_release);
    // Preserve the process name used by run_w1.sh's pgrep -x w1_server.
    RegisterCurrentThread("w1_server");
    struct sigaction action{};
    action.sa_sigaction = HandleFatalSignal;
    action.sa_flags = SA_SIGINFO | SA_ONSTACK;
    sigemptyset(&action.sa_mask);
    for (int signal_number : fatal_signals) {
        Check(sigaction(signal_number, &action, nullptr) == 0, "sigaction fatal signal");
    }
    RecordProcessMaps();
    std::fprintf(stderr, "[crash] log=%s; core destination follows kernel.core_pattern: %s",
                 file.c_str(), pattern.c_str());
    if (limit.rlim_cur == 0) std::fprintf(stderr, "[crash] WARNING: core hard limit is zero; direct core files disabled.\n");
    std::fprintf(stderr, "[crash] Core collection must be verified on the deployment host; SIGKILL/RCU stalls are not captured.\n");
}
}  // namespace robot_server_lib::crash_diagnostics
