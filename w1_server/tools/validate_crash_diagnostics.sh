#!/usr/bin/env bash
# Builds/runs only the small standalone probe. Never starts robot processes.
set -euo pipefail
repo_dir=$(cd "$(dirname "$0")/../.." && pwd)
test_build_dir=$(mktemp -d /tmp/w1-crash-validation-build.XXXXXX)

# Apply limits BEFORE launching any compiler/test subprocess. These limits
# are inherited. The death-test runner further limits each probe to 256 MiB.
ulimit -v 524288
ulimit -t 120
ulimit -c 0
export CMAKE_BUILD_PARALLEL_LEVEL=1
printf 'Standalone build directory: %s\n' "$test_build_dir"
cmake -S "$repo_dir/robot_server_lib/test/crash_diagnostics" -B "$test_build_dir"
cmake --build "$test_build_dir" --parallel 1
ctest --test-dir "$test_build_dir" --output-on-failure -V
printf 'Probe retained at: %s/crash_probe\n' "$test_build_dir"
