#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source ~/.bashrc
source $DIR/../setup.sh
nohup roslaunch movo_ros robotiq_node.launch &>/dev/null &
