#!/bin/bash

SCRIPTS_PATH="$(dirname "$(realpath "$0")")"
BUILD_PATH=$SCRIPTS_PATH/../../build
BUILD_PATH=$(realpath $BUILD_PATH)

#create build path if necessary
if ! [ -d $BUILD_PATH ]
then
    mkdir -p $BUILD_PATH
fi

cd $BUILD_PATH
cmake ..
make
