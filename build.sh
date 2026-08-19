#!/usr/bin/env bash
set -e

os="$(uname -s)"

mkdir_and_build () {
    mkdir -p build
    cd build
    cmake ..
    if [ "$os" = "Darwin" ]; then
        cmake --build . -j "$(sysctl -n hw.ncpu)"
    else
        cmake --build . -j "$(nproc)"
    fi
}

cd "$(dirname "${BASH_SOURCE[0]}")"

if [ "$os" = "Darwin" ]; then
    # macOS: only MotorIP / MotorEthL2 are supported (MotorCAN is a compile-stub,
    # local USB/UART transports are unavailable). CLI11/json are fetched by cmake
    # and libudev is not used. Requires cmake and the Xcode command line tools.
    if ! command -v cmake >/dev/null 2>&1; then
        echo "cmake not found. Install it, e.g.: brew install cmake"
        exit 1
    fi
    mkdir_and_build
else
    build_deps="build-essential cmake libudev-dev git python3-dev"

    check_build_deps () {
        for dep in $@; do
            if ! dpkg-query -s $dep &>/dev/null; then
                echo $dep not installed
                return 1;
            fi
        done
        return 0
    }

    install_build_deps () {
        sudo apt update
        sudo apt install -y $build_deps
    }

    if ! check_build_deps $build_deps; then
        install_build_deps $build_deps
    fi
    mkdir_and_build
fi
