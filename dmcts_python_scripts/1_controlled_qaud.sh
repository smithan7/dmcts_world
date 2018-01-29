#!/bin/bash

n_agents=1
n_nodes=$2
param=$3
 
my_pid=$$
echo "My process ID is $my_pid"

echo "Sourcing ~/catkin_ws/devel/setup.bash"
source ~/catkin_ws/devel/setup.bash &
pid=$!

echo "Launching roscore..."
roscore &
pid="pid $!"
sleep 5s

echo "Launching Gazebo..."
roslaunch gazebo_ros empty_world.launch & #willowgarage_world.launch &
pid="$pid $!"
sleep 5s

declare -a xs=(-25 25 -25);
declare -a ys=(-25 25 25);
echo "launching hector quadrotors"
ai=0
roslaunch hector_quadrotor_gazebo spawn_n_quadrotor.launch agent_index:=$ai x_start:=${xs[ai]} y_start:=${ys[ai]} &
pid="$pid $!"
sleep 5s

#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch num_agents:=$n_agents num_nodes:=$n_nodes param_number:=$param &
pid="$pid $!"
sleep 5s

echo "launching dmcts quad nodes"
roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='greedy_completion_reward' &
#roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward' &
#roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward_impact_optimal' &
pid="$pid $!"
sleep 5s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h
