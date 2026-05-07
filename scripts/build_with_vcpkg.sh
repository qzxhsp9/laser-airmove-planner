#!/usr/bin/env bash
set -euo pipefail

: "${VCPKG_ROOT:?Please set VCPKG_ROOT to your vcpkg directory}"

cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build -j
