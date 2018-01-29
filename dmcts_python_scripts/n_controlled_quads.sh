#!/bin/bash

n_agents=$1
n_nodes=$2
param=$3
coord_method=$4
 
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

declare -a xs=(-25 25 -25 25 25 0 0 -25);
declare -a ys=(-25 25 25 -25 0 25 -25 0);
echo "launching hector quadrotors"
if [ $n_agents -gt 1 ]
then
    start=0
    for ((ai=0; ai<n_agents; ai++))
    do
        roslaunch hector_quadrotor_gazebo spawn_n_quadrotor.launch agent_index:=$ai x_start:=${xs[ai]} y_start:=${ys[ai]} &
        pid="$pid $!"
        sleep 5s
    done
else
    roslaunch hector_quadrotor_gazebo spawn_n_quadrotor.launch agent_index:=0 x_start:=${xs[0]} y_start:=${ys[0]} &
    pid="$pid $!"
    sleep 5s
fi  

#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch num_agents:=$n_agents num_nodes:=$n_nodes param_number:=$param &
pid="$pid $!"
sleep 5s

echo "launching dmcts quad nodes"
if [ $n_agents -gt 1 ]
then
    start=0
    for ((ai=0; ai<n_agents; ai++))
    do
        if [ $coord_method = 0 ]
        then
            roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='greedy_completion_reward' &
        fi
        
        if [ $coord_method = 1 ]
        then
            roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward' &
        fi
        
        if [ $coord_method = 2 ]
        then
            roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward_impact_optimal' &
        fi
        pid="$pid $!"
        sleep 5s
    done
else
    if [ $coord_method = 0 ]
    then
        roslaunch dmcts dmcts_n.launch agent_index:=0 num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='greedy_completion_reward' &
    fi
    
    if [ $coord_method = 1 ]
    then
        roslaunch dmcts dmcts_n.launch agent_index:=0 num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward' &
    fi
    
    if [ $coord_method = 2 ]
    then
        roslaunch dmcts dmcts_n.launch agent_index:=0 num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward_impact_optimal' &
    fi
    pid="$pid $!"
    sleep 5s
    pid="$pid $!"
    sleep 5s
fi

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h
