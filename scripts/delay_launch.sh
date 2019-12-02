#!/bin/bash
#
# Copyright (c) 2016, Masaru Morita
# All rights reserved

if [ $# -lt 3 -o "$1" = "-h" ]; then
    echo "Starting Delayed"
else
    echo "Starting delayed by $1 seconds"
    sleep $1
    shift
        echo "Running roslaunch $@"
    roslaunch $@
fi