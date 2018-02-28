#!/bin/bash

n_agents=$1
n_nodes=$2
n_trials=$3

my_pid=$$
echo "My process ID is $my_pid"
echo "  n_agents: $n_agents"
echo "  n_nodes: $n_nodes"
echo "  n_trials: $n_trials"

for ((t=1; t<n_trials+1; t++))
do
	echo "***********************************************************************************************************"
	echo "*****************************************  Starting trial $t **********************************************"
	echo "***********************************************************************************************************"
    ./n_controlled_fake_quads.sh $n_agents $n_nodes $t "greedy_completion_reward"
	echo "***********************************************************************************************************"
	echo "*****************************************  Finished Greedy trial $t ***************************************"
	echo "***********************************************************************************************************"
	sleep 10s
	./n_controlled_fake_quads.sh $n_agents $n_nodes $t "mcts_task_by_completion_reward_gradient"
	echo "***********************************************************************************************************"
	echo "*****************************************  Finished MCTS trial $t *****************************************"
	echo "***********************************************************************************************************"

done
