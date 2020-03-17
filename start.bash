#!/bin/bash

file=""

if [ $1 == "m" ]; then
    file="measurement.py"
elif [ $1 == "p" ]; then
    file="processing.py"
elif [ $1 == "v" ]; then
    file="view.py"
fi

if [ "$file" != "" ]; then
    source devel/setup.bash
    rosrun sensors $file
fi

