#!/usr/bin/env bash
##
## BSD 3-Clause License
##
## This file is part of the RootBA project.
## https://github.com/NikolausDemmel/rootba
##
## Copyright (c) 2021, Nikolaus Demmel.
## All rights reserved.
##


# given folder with rootba_config_*.toml, run optimization for each config in
# corresponding subfolder

set -e
set -x

# number of logical cores on linux and macos
NUM_CORES=`(which nproc > /dev/null && nproc) || sysctl -n hw.logicalcpu || echo 1`

echo "Running on '`hostname`', nproc: $NUM_CORES"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# loop over all arguments, and in each folder find configs and run them
for FOLDER in "$@"
do

    pushd "$FOLDER"

    FILE_PATTERN='rootba_config_*.toml'
    FILE_REGEX='rootba_config_(.*)\.toml'

    DATE=`date +'%Y%m%d-%H%M%S'`
    mkdir -p $DATE

    declare -a RUN_DIRS=()

    for f in `find . -name "$FILE_PATTERN" -type f | sort`; do
        if [[ `basename $f` =~ $FILE_REGEX ]]; then
            RUN_DIR=${DATE}/`dirname $f`/${BASH_REMATCH[1]}
            echo "Creating run with config $f in $RUN_DIR"
            mkdir -p "$RUN_DIR"
            cp $f "$RUN_DIR"/rootba_config.toml
            echo "Created" > "$RUN_DIR"/status.log
            RUN_DIRS+=($RUN_DIR)
        else
            echo "Skipping $f"
        fi
    done

    for RUN_DIR in "${RUN_DIRS[@]}"; do
        echo "Starting run in $RUN_DIR"
        "$SCRIPT_DIR"/run-one.sh "$RUN_DIR" || true
    done

    popd

done
