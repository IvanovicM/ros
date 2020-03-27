#!/bin/bash

run_gazebo() {
    export TURTLEBOT3_MODEL=burger
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
}

run_rviz() {
    export TURTLEBOT3_MODEL=burger
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
}

maybe_run_builtin_pkg() {
    if [ $1 == "gazebo" ]; then
        run_gazebo
        return
    fi
    if [ $1 == "rviz" ]; then
        run_rviz
        return
    fi
    echo "Illegal ros package and node"
}

get_package() {
    pkg=""
    if [ $1 == "s" ]; then
        pkg="sensors"
    elif [ $1 == "k" ]; then
        pkg="kinematics"
    fi
}

get_file() {
    file=""
    if [ $1 == "m" ]; then
        file="measurement.py"
    elif [ $1 == "p" ]; then
        file="processing.py"
    elif [ $1 == "v" ]; then
        file="view.py"
    elif [ $1 == "r" ]; then
        file="record.py"
    elif [ $1 == "a" ]; then
        file="diffaction.py"
    elif [ $1 == "mc" ]; then
        file="movement_control.py"
    elif [ $1 == "dg" ]; then
        file="dummy_gazebo.py"
    fi
}

maybe_run_custom_pkg() {
    get_package $1
    get_file $2

    if [ "$pkg" != "" ] && [ "$file" != "" ]; then
        source devel/setup.bash
        rosrun $pkg $file
    else
        echo "Illegal ros package and node"
    fi
}

check_params_and_run() {
    if [ "$#" == 1 ]; then
        maybe_run_builtin_pkg $@
        return
    fi
    if [ "$#" == 2 ]; then
        maybe_run_custom_pkg $@
        return
    fi

    echo "Illegal number of parameters"
}

check_params_and_run $@
