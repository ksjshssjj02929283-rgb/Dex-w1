#!/bin/bash

ARCH=$(uname -m)

if [ "$ARCH" = "x86_64" ]; then
    bash "$(dirname "$0")/w1_server/tools/run_plotjuggler_x86_64.sh" "$@"
elif [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    bash "$(dirname "$0")/w1_server/tools/run_plotjuggler_arm64.sh" "$@"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi
