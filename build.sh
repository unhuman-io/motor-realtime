#!/usr/bin/bash -e

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

mkdir_and_build () {
    mkdir -p build
    cd build
    cmake ..
    cmake --build . -j `nproc`
}


cd $(dirname ${BASH_SOURCE[0]})

if ! check_build_deps $build_deps; then
    install_build_deps $build_deps
fi
mkdir_and_build