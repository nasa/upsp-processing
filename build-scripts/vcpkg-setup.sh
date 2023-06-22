#!/bin/bash

if command -v yum &> /dev/null; then
    yum install -y \
        perl-IPC-Cmd \
        zip
fi

if [ -e /opt/vcpkg ]; then
    echo "rm -rf /opt/vcpkg"
    rm -rf /opt/vcpkg
fi

git clone \
    --depth 1 \
    --branch "2023.04.15" \
    https://github.com/Microsoft/vcpkg.git \
    /opt/vcpkg

pushd .

cd /opt/vcpkg
echo "vcpkg version: '$(git describe)'"
./bootstrap-vcpkg.sh -disableMetrics
./vcpkg install --triplet=x64-linux-dynamic \
  imath[core] \
  pybind11[core]

popd
