#!/bin/bash
set -e

# Limit the thread number for compiling
# thread_count=$(nproc --all)
# threads_for_compilation=$((thread_count - 6))
# export MAKEFLAGS="-j $threads_for_compilation"

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
        -Wall -Wextra -Wpedantic \
        --parallel-workers 2