#!/usr/bin/env bash
set -ex

#######################
# 终端1：RealSense
#######################
gnome-terminal -- bash -ic '
set -x
cd /home/young/realsense_ws
set +x
source install/local_setup.bash
set -x
ros2 launch realsense2_camera rs_launch.py
exec bash
'

sleep 1

#######################
# 终端2：controller
#######################
gnome-terminal -- bash -ic '
set -x
conda activate py310
cd ~/workspace/dexe_robot_client
set +x
source dexe_interfaces/install/setup.bash
set -x
cd examples/planner_controller
python3 w1_service_manager.py --sim
exec bash
'

sleep 1

# #######################
# # 终端3：planner
# #######################
gnome-terminal -- bash -ic '
set -x
conda activate py310
cd ~/workspace/dexe_robot_client
set +x
source dexe_interfaces/install/setup.bash
set -x
cd examples/planner_controller
python3 w1_planner_example4.py
exec bash
'
