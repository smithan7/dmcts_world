#!/bin/bash
my_pid=$$
echo "My process ID is $my_pid"

declare -a n_agents=(1 2 3 4);
declare -a n_nodes=(20 30 40 50);
declare -a params=(-1 -1 -1 -1)

echo "launching test"
start=0
for ((ai=0; ai<n_agents; ai++))
do
    sh ./n_controlled_quads.sh 1 20 -1 0 #${n_agents[ai]} ${n_nodes[ai]} ${param[ai]} 0
    pid2=pidof n_controlled_quads.sh 
    pid="$pid $!"
    sleep 5m
    kill pid2
    
    ./n_controlled_quads.sh ${n_agents[ai]} ${n_nodes[ai]} ${param[ai]} 1
    pid="$pid $!"
    sleep 10m
done


trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h
