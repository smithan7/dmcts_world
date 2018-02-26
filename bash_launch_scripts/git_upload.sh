#!/bin/bash

reason=1
sleep_time='5s'
commit_time='1s'

echo "pushing costmap_bridge**********************************************************************"
cd ~/catkin_ws/src/costmap_bridge
git add -A
git commit -m "$reason"
sleep $commit_time
git push
sleep $sleep_time

echo "pushing custom messages**********************************************************************"
cd ~/catkin_ws/src/custom_messages
git add -A
git commit -m "$reason"
git push
sleep $sleep_time

echo "pushing dmcts**********************************************************************"
cd ~/catkin_ws/src/dmcts
git status
git add -A
git commit -m "$reason"
git push
sleep $sleep_time

echo "pushing dmcts_world **********************************************************************"
cd ~/catkin_ws/src/dmcts_world
git status
git add -A
git commit -m "$reason"
git push
sleep $sleep_time

echo "pushing fake dmcts quad**********************************************************************"
cd ~/catkin_ws/src/fake_dmcts_quad
git status
git add -A
git commit -m "$reason"
git push
sleep $sleep_time

echo "pushing gps_to_local_bridge**********************************************************************"
cd ~/catkin_ws/src/gps_to_local_bridge
git status
git add -A
git commit -m "$reason"
git push
sleep $sleep_time

echo "pushing my_quad_controller**********************************************************************"
cd ~/catkin_ws/src/my_quad_controller
git status
git add -A
git commit -m "$reason"
git push
sleep $sleep_time

echo "pushing xbee_bridge**********************************************************************"
cd ~/catkin_ws/src/xbee_bridge
git status
git add -A
git commit -m "$reason"
git push
sleep $sleep_time

echo "Complete"
