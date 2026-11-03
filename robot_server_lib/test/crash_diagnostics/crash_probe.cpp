#include "crash_diagnostics.h"

#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace diagnostics = robot_server_lib::crash_diagnostics;
void* bad_address = nullptr;
std::atomic<bool> ready{false};

__attribute__((noinline)) void CrashLeaf() {
    *static_cast<volatile int*>(bad_address) = 42;
}

__attribute__((noinline)) void CrashMiddle() {
    CrashLeaf();
    std::atomic_signal_fence(std::memory_order_seq_cst);
}

// Volatile function pointer and post-call buffer access prevent tail-call
// elimination. A small pthread stack makes the overflow test deterministic.
void Overflow(unsigned depth);
void (*volatile recurse)(unsigned) = Overflow;
__attribute__((noinline)) void Overflow(unsigned depth) {
    volatile char buffer[4096];
    buffer[depth % sizeof(buffer)] = static_cast<char>(depth);
    recurse(depth + 1);
    if (buffer[depth % sizeof(buffer)] == 127) _exit(2);
}

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    const std::string mode = argv[1];
    // Reproduce capability-triggered non-dumpability without requiring root.
    if (prctl(PR_SET_DUMPABLE, 0) != 0) return 3;
    diagnostics::Initialize(argv[2], "standalone-crash-probe");
    if (prctl(PR_GET_DUMPABLE) != 1) return 4;
    // Don't flood the host core collector during repeated death tests. GDB
    // generate-core-file is tested separately and does not use RLIMIT_CORE.
    rlimit limit{};
    getrlimit(RLIMIT_CORE, &limit);
    limit.rlim_cur = 0;
    setrlimit(RLIMIT_CORE, &limit);
    bad_address = mmap(nullptr, 4096, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (bad_address == MAP_FAILED) return 5;
    diagnostics::RecordProcessMaps();

    if (mode == "worker" || mode == "unregistered" || mode == "core") {
        std::thread worker([&]() {
            if (mode != "unregistered") diagnostics::RegisterCurrentThread("probe_worker");
            while (!ready.load()) std::this_thread::yield();
            CrashMiddle();
        });
        ready.store(true);
        worker.join();
    } else if (mode == "overflow") {
        pthread_attr_t attributes;
        pthread_attr_init(&attributes);
        pthread_attr_setstacksize(&attributes, 64 * 1024);
        pthread_t worker;
        const int error = pthread_create(&worker, &attributes, [](void*) -> void* {
            diagnostics::RegisterCurrentThread("probe_overflow");
            Overflow(0);
            return nullptr;
        }, nullptr);
        pthread_attr_destroy(&attributes);
        if (error != 0) return 6;
        pthread_join(worker, nullptr);
    } else if (mode == "locked") {
        // A held application lock cannot prevent the independent crash writer.
        std::mutex application_mutex;
        std::lock_guard lock(application_mutex);
        CrashMiddle();
    } else if (mode == "segv") {
        CrashMiddle();
    } else if (mode == "abort") {
        std::abort();
    } else if (mode == "exception") {
        throw std::runtime_error("uncaught probe exception");
    } else if (mode == "bus") {
        raise(SIGBUS);
    } else if (mode == "ill") {
        raise(SIGILL);
    } else if (mode == "fpe") {
        raise(SIGFPE);
    } else if (mode == "term") {
        raise(SIGTERM);
    } else if (mode == "int") {
        raise(SIGINT);
    } else if (mode == "kill") {
        raise(SIGKILL);
    } else {
        return 7;
    }
    return 8;  // A fatal signal must not turn into a normal exit.
}
