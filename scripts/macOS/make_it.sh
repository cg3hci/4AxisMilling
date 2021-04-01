#!/bin/bash

SCRIPTS_PATH=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

sh $SCRIPTS_PATH/0_install_macos.sh
sh $SCRIPTS_PATH/1_build.sh
sh $SCRIPTS_PATH/2_run_paper_results.sh
