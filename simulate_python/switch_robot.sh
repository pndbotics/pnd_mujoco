#!/bin/bash
if [ "$1" == "adam_u" ] || [ "$1" == "adam_lite" ] || [ "$1" == "adam_sp" ]; then
    cd ~/pnd_sdk_python/pnd_mujoco/simulate_python/
    sed -i "s/ROBOT = \".*\"/ROBOT = \"$1\"/" config.py
    echo "Switched to robot: $1"
    echo "Current config:"
    grep ROBOT config.py
else
    echo "Usage: $0 [adam_u|adam_lite|adam_sp]"
    echo "Current robot:"
    cd ~/pnd_sdk_python/pnd_mujoco/simulate_python/
    grep ROBOT config.py
fi
