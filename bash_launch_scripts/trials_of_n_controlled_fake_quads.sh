#!/bin/bash

n_agents=$1
n_nodes=$2
n_trials=$3

score_run=false
gazebo_obstacles=false
cruising_speed=2.0
use_xbee=false
 
my_pid=$$
echo "My process ID is $my_pid"
echo "  n_agents: $n_agents"
echo "  n_nodes: $n_nodes"
echo "  n_trials: $n_trials"

for ((t=0; t<n_trials; t++))
do
	echo "***********************************************************************************************************"
	echo "*****************************************  Starting trial $t **********************************************"
	echo "***********************************************************************************************************"
    ./n_controlled_fake_quads.sh $n_agents $n_nodes 34 "greedy_completion_reward"
	echo "***********************************************************************************************************"
	echo "*****************************************  Finished trial $t **********************************************"
	echo "***********************************************************************************************************"
	sleep 10s

done


trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h
