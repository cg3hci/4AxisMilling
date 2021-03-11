#!/bin/bash

SCRIPTS_PATH="$(dirname "$(realpath "$0")")"
BUILD_PATH=$SCRIPTS_PATH/../../build
BUILD_PATH=$(realpath $BUILD_PATH)
RESULTS_PATH=$SCRIPTS_PATH/../../results
MODELS_PATH=$SCRIPTS_PATH/../../misc/input_meshes/

#create results path if necessary
if ! [ -d $RESULTS_PATH ]
then
    mkdir -p $RESULTS_PATH
fi

cd $BUILD_PATH
./fourAxisMilling -i=$MODELS_PATH/kitten.obj -o=$RESULTS_PATH/kitten --model_length=70 --stock_length=88 --stock_diameter=72
