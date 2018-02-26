#!/bin/bash
 
my_pid=$$
echo "My process ID is $my_pid"

echo "pushing dmcts_world"
cd ~/catkin_ws/src/dmcts_world &
git status &
git add -A &
git commit -m "automatic upload" &
git push &
pid=$!

sleep 1s

echo "pushing dmcts"
cd ~/catkin_ws/src/dmcts &
git status &
git add -A &
git commit -m "automatic upload" &
git push &
pid=$!
sleep 1s


