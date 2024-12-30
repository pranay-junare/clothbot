#! /bin/bash

# tmux check if session exists:
if [[ $(tmux ls | grep ROS_RX) ]]; then
    tmux kill-session -t ROS_RX
fi

tmux new-session -d -s "ROS_RX" \; \
    send-keys "roslaunch dual_arm zeus_bringup.launch" C-m \; \
    split-window -h \; \
    send-keys "sleep 5; python3 ./SparkNode.py" C-m \; \
    split-window -v \; \
    send-keys "sleep 10; python3 ./run.py" C-m \; \
    split-window -v \; \
    send-keys "sleep 5; rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNodeRight.py /tmp/ttyThunder" C-m \; \
    split-window -v \; \
    send-keys "sleep 5; rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNodeLeft.py /tmp/ttyLightning" C-m \; \
    split-window -v \; \
    send-keys "sleep 8; rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleControllerLeft.py" C-m \; \
    split-window -v \; \
    send-keys "sleep 8; rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleControllerRight.py" C-m \; \
    select-layout tiled \; \
    attach-session -t "ROS_RX"
