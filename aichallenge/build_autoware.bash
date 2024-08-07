#!/bin/bash
pip install setuptools==58.2.0

if [[ ${1} == "clean" ]]; then
    echo "clean build"
    rm -r ./workspace/build/* ./workspace/install/*
fi

cd ./workspace || exit
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ../