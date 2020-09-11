#!/bin/bash

docker exec -it --user user dexterous_hand_real_hw /ros_entrypoint.sh bash -c 'source /home/user/projects/shadow_robot/base_deps/devel/setup.bash;source /home/user/projects/shadow_robot/base/devel/setup.bash;rosrun sr_example sr_store_hand.py'

