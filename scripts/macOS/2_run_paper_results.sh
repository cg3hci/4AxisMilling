#!/bin/bash

SCRIPTS_PATH=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
BUILD_PATH=$SCRIPTS_PATH/../../build
RESULTS_PATH=$SCRIPTS_PATH/../../results
MODELS_PATH=$SCRIPTS_PATH/../../misc/input_meshes/

#create results path if necessary
if ! [ -d $RESULTS_PATH ]
then
    mkdir -p $RESULTS_PATH
fi

cd $BUILD_PATH
# fabricated results
./fourAxisMilling -i=$MODELS_PATH/kitten.obj -o=$RESULTS_PATH/kitten --model_height=70 --stock_diameter=72 --stock_length=88
./fourAxisMilling -i=$MODELS_PATH/batman.obj -o=$RESULTS_PATH/batman --model_height=70 --stock_diameter=72 --stock_length=92
./fourAxisMilling -i=$MODELS_PATH/david.obj -o=$RESULTS_PATH/david --model_height=54 --stock_diameter=62 --stock_length=86
./fourAxisMilling -i=$MODELS_PATH/buddha.obj -o=$RESULTS_PATH/buddha --model_height=70 --stock_diameter=72 --stock_length=86 --n_smooth_iterations=750
