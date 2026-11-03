#!/usr/bin/env bash
# Offline only; never attaches to a running robot process.
set -euo pipefail
if [[ $# -ne 2 ]]; then
  echo "Usage: bash analyze_core.sh /path/to/matching/executable /path/to/core > backtrace.txt" >&2
  exit 2
fi
ulimit -v 1048576
ulimit -t 120
ulimit -c 0
ulimit -Hc 0
export MALLOC_ARENA_MAX=2
exec gdb -nx -q -batch \
  -iex 'maintenance set worker-threads 1' \
  -iex 'set auto-load safe-path /nonexistent' \
  -iex 'set debuginfod enabled off' \
  -ex 'set pagination off' \
  -ex 'info files' \
  -ex 'info threads' \
  -ex 'thread apply all bt full' \
  -ex 'info registers' \
  -- "$1" "$2"
