#!/bin/bash

cd system/unity/
./system_bring_up.sh &
sleep 5
source ai_module/devel/setup.sh
roslaunch dummy_vlm dummy_vlm.launch
