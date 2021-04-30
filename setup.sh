#!/bin/bash

SCRIPTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export OPPT_RESOURCE_PATH="${OPPT_RESOURCE_PATH}:${SCRIPTDIR}/models:$SCRIPTDIR/plugins:"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${SCRIPTDIR}/models:"

# Find the local ROS_IP
addr=$(hostname --all-ip-addresses || hostname -I)
IFS=' ' read -ra ADDR <<< "$addr"
for i in "${ADDR[@]}"; do
    if [[ $i == "10.66.171"* ]]; then
       ip=$i     
       break
    fi      
done

export ROS_MASTER_URI=http://10.66.171.1:11311/
export ROS_IP=$ip
