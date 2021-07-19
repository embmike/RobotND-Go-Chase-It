#!/bin/sh

# Application setup:
# Go chase it
# pwd=./RobotND-Go-Chase-It

xterm -e " source devel/setup.bash;roslaunch my_robot world.launch " &

sleep 5
xterm -e " source devel/setup.bash;roslaunch ball_chaser ball_chaser.launch " &

sleep 5 
xterm -e " source devel/setup.bash;rosrun rqt_image_view rqt_image_view "

