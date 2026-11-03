#pragma once

#include <string>

namespace robot_server_lib::crash_diagnostics {

// Call once, before creating threads. Throws on initialization failure.
// Opens a private, unique crash log, enables this process's dumpability and
// raises its soft core limit to the hard limit. Never changes system sysctls.
void Initialize(const std::string& directory, const std::string& version);

// Call at thread entry, before entering real-time scheduling. A no-op when
// Initialize was not called (e.g. sim_server). Installs a per-thread alternate
// signal stack and records the Linux TID. Throws if setup fails.
void RegisterCurrentThread(const char* name);

// Normal-context snapshot, useful after shared libraries have been loaded.
void RecordProcessMaps();

}  // namespace robot_server_lib::crash_diagnostics
