#!/bin/bash

set -e

# check dependecies (cmake)
if ! command -v cmake >/dev/null 2>&1 ; then
    echo "Error: cmake not found on system!"
    exit 1
fi

# Build the agent
if [ -d "build" ]; then
    echo "Status: Removing old build directory..."
    rm -r build
    if [$? -ne 0]; then
        echo "Error: Could not remove old build directory! Please remove manually."
        exit 1
    fi
fi

mkdir -p build
cd build

echo "Status: Building agent..."
cmake ..
if ! command -v nproc >/dev/null 2>&1 ; then
    make -j4
else
    make -j$(nproc)
fi