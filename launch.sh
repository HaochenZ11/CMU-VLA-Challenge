#!/bin/bash

cd system/unity/
./system_bring_up.sh &
sleep 5
cd ../../
source ai_module/devel/setup.bash
roslaunch dummy_vlm dummy_vlm.launch
