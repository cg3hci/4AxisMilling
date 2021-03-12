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

echo "Computing fabricated results in parallel..."
# fabricated results
./fourAxisMilling -i=$MODELS_PATH/kitten.obj -o=$RESULTS_PATH/kitten --model_height=70 --stock_diameter=72 --stock_length=88 > $RESULTS_PATH/kitten.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/batman.obj -o=$RESULTS_PATH/batman --model_height=70 --stock_diameter=72 --stock_length=92 > $RESULTS_PATH/batman.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/david.obj -o=$RESULTS_PATH/david --model_height=54 --stock_diameter=62 --stock_length=86 > $RESULTS_PATH/david.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/buddha.obj -o=$RESULTS_PATH/buddha --model_height=70 --stock_diameter=72 --stock_length=86 --prefiltering_smooth_iters=750 > $RESULTS_PATH/buddha.log 2>&1 &

wait
echo "Fabricated results computed."

echo "Computing first four results..."
./fourAxisMilling -i=$MODELS_PATH/moai.obj -o=$RESULTS_PATH/moai --just_segmentation > $RESULTS_PATH/moai.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/max_planck.obj -o=$RESULTS_PATH/max_planck --just_segmentation > $RESULTS_PATH/max_planck.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/egea.obj -o=$RESULTS_PATH/egea --just_segmentation > $RESULTS_PATH/egea.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/pensatore.obj -o=$RESULTS_PATH/pensatore --just_segmentation > $RESULTS_PATH/pensatore.log 2>&1 &

wait
echo "First four results computed."

echo "Computing second four results..."
./fourAxisMilling -i=$MODELS_PATH/dea.obj -o=$RESULTS_PATH/dea --just_segmentation --prefiltering_smooth_iters=750 > $RESULTS_PATH/dea.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/maneki_neko.obj -o=$RESULTS_PATH/maneki_neko --just_segmentation > $RESULTS_PATH/maneki_neko.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/eros.obj -o=$RESULTS_PATH/eros --just_segmentation > $RESULTS_PATH/eros.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/chinese_lion.obj -o=$RESULTS_PATH/chinese_lion --just_segmentation > $RESULTS_PATH/chinese_lion.log 2>&1 &


wait
echo "Second four results computed."

echo "Computing third four results..."
./fourAxisMilling -i=$MODELS_PATH/faget_statue.obj -o=$RESULTS_PATH/faget_statue --just_segmentation > $RESULTS_PATH/faget_statue.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/BU.obj -o=$RESULTS_PATH/BU --just_segmentation > $RESULTS_PATH/BU.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/3holes.obj -o=$RESULTS_PATH/3holes --model_height=80 --prefiltering_smooth_iters=0 --just_segmentation > $RESULTS_PATH/3holes.log 2>&1 &
./fourAxisMilling -i=$MODELS_PATH/woman_statue.obj -o=$RESULTS_PATH/woman_statue --just_segmentation > $RESULTS_PATH/woman_statue.log 2>&1 &

wait
echo "Third four results computed."
