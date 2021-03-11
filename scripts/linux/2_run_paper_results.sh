#!/bin/bash

SCRIPTS_PATH="$(dirname "$(realpath "$0")")"
BUILD_PATH=$SCRIPTS_PATH/../../build
BUILD_PATH=$(realpath $BUILD_PATH)
RESULTS_PATH=$SCRIPTS_PATH/../../results

#create results path if necessary
if ! [ -d $RESULTS_PATH ]
then
    mkdir -p $RESULTS_PATH
fi
