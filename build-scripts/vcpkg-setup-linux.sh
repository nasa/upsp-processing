#!/bin/bash

VCPKG_ROOT=/opt/vcpkg
VCPKG_TRIPLET="x64-linux-dynamic"

echo "Settings up vcpkg in '$VCPKG_ROOT' ..."

yum install -y \
    perl-IPC-Cmd \
    zip

if [ ! -e "$VCPKG_ROOT" ]; then
    git clone \
        --depth 1 \
        --branch "2023.04.15" \
        https://github.com/Microsoft/vcpkg.git \
        $VCPKG_ROOT

    pushd .
    cd $VCPKG_ROOT
    echo "vcpkg version: '$(git describe)'"
    ./bootstrap-vcpkg.sh -disableMetrics
    popd
fi

pushd .
cd $VCPKG_ROOT
./vcpkg install --triplet=$VCPKG_TRIPLET \
  imath[core] \
  pybind11[core]

popd
