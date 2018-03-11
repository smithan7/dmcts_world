#!/bin/bash

n_agents=$1
n_nodes=$2
param=$3
coord_method=$4
score_run=false
gazebo_obstacles=false
p_task_initially_active=0.40
cruising_speed=2.0
 
my_pid=$$
echo "My process ID is $my_pid"
echo "  n_agents: $n_agents"
echo "  n_nodes: $n_nodes"
echo "  param: $param"
echo "  coord_method: $coord_method"

echo "Sourcing ~/catkin_ws/devel/setup.bash"
source ~/catkin_ws/devel/setup.bash &
pid=$!

echo "Launching roscore..."
roscore &
pid="pid $!"
sleep 1s

echo "setting sim_time true"
#rosparam set /use_sim_time true &
#pid="$pid $!"
#sleep 1s

#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch num_agents:=$n_agents num_nodes:=$n_nodes param_number:=$param coord_method:=$coord_method  score_run:=$score_run gazebo_obstacles:=$gazebo_obstacles p_task_initially_active:=$p_task_initially_active &
pid="$pid $!"
sleep 1s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h
