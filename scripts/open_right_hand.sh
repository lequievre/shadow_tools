docker cp ./sr_open_hand.py dexterous_hand_real_hw:/home/user/projects/shadow_robot/base/src/sr_interface/sr_example/scripts/sr_example/hand_examples
docker exec -it --user user dexterous_hand_real_hw /ros_entrypoint.sh bash -c 'source /home/user/projects/shadow_robot/base_deps/devel/setup.bash;source /home/user/projects/shadow_robot/base/devel/setup.bash; chmod +x /home/user/projects/shadow_robot/base/src/sr_interface/sr_example/scripts/sr_example/hand_examples/sr_open_hand.py ;rosrun sr_example sr_open_hand.py'
