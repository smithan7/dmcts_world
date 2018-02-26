#!/bin/bash

n_agents=$1
n_nodes=$2
param=$3
coord_method=$4
my_index=$5
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

declare -a xs=(-25 25 -25 25 25 0 0 -25);
declare -a ys=(-25 25 25 -25 0 25 -25 0);
declare -a zs=(5 7.5 10 12.5 15 17.5 20 22.5);
declare -a pay_obs_costs=(false false false false false false false false)

#echo "setting sim_time true"
#rosparam set /use_sim_time true &
#pid="$pid $!"
#sleep 1s


#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts fake quad node"
roslaunch fake_dmcts_quad fake_dmcts_quad.launch agent_index:=$my_index agent_altitude:=${zs[my_index]} start_x:=${xs[my_index]} start_y:=${ys[my_index]} cruising_speed:=$cruising_speed & 
    pid="$pid $!"
    sleep 1s

echo "launching dmcts quad node"
    roslaunch dmcts dmcts_n.launch agent_index:=$my_index num_agents:=$n_agents num_nodes:=$n_nodes coord_method:=$coord_method  desired_altitude:=${zs[my_index]} p_task_initially_active:=$p_task_initially_active pay_obstacle_costs:=${pay_obs_costs[my_index]} not_simulation:=false cruising_speed:=$cruising_speed param_number:=$param & 
#roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='greedy_completion_reward' &
#roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward' &
#roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward_impact_optimal' &
    pid="$pid $!"
    sleep 2s


trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h