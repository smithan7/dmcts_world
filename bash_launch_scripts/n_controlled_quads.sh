#!/bin/bash

n_agents=$1
n_nodes=$2
param=$3
coord_method=$4
score_run=true
p_task_initially_active=$5
search_depth=$6
beta=$7
alpha=$8
gamma=$9
epsilon=${10}
min_sampling_threshold=${11}
gazebo_obstacles=true
cruising_speed=1.0
use_xbee=false
end_time=120.0
way_point_tol=5.0
 
my_pid=$$
echo "My process ID is $my_pid"
echo "  n_agents: $n_agents"
echo "  n_nodes: $n_nodes"
echo "  param: $param"
echo "  coord_method: $coord_method"
echo "  p_task_initially_active: $p_task_initially_active"
echo "  search_depth: $search_depth"
echo "  beta: $beta"
echo "  alpha: $alpha"
echo "  gamma: $gamma"
echo "  epsilon: $epsilon"
echo "  min_sampling_threshold: $min_sampling_threshold"
echo "  way_point_tol: $way_point_tol"

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
declare -a zs=(5 7.5 10 12.5 15 17.5 20 22.5);
declare -a pay_obs_costs=(false false false false false false false false)

echo "launching hector quadrotors"
for ((ai=0; ai<n_agents; ai++))
do
    roslaunch hector_quadrotor_gazebo spawn_n_quadrotor.launch agent_index:=$ai x_start:=${xs[ai]} y_start:=${ys[ai]} &
    pid="$pid $!"
    sleep 5s
done 

#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts quad nodes"
for ((ai=0; ai<n_agents; ai++))
do
    roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:=$coord_method  desired_altitude:=${zs[ai]} p_task_initially_active:=$p_task_initially_active pay_obstacle_costs:=${pay_obs_costs[ai]} not_simulation:=true cruising_speed:=$cruising_speed cruising_speed:=$cruising_speed param_number:=$param use_xbee:=$use_xbee alpha:=$alpha beta:=$beta gamma:=$gamma epsilon:=$epsilon search_depth:=$search_depth min_sampling_threshold:=$min_sampling_threshold end_time:=$end_time way_point_tol:=$way_point_tol & 
    #roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='greedy_completion_reward' &
    #roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward' &
    #roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward_impact_optimal' &
    pid="$pid $!"
    sleep 5s
done

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch num_agents:=$n_agents num_nodes:=$n_nodes param_number:=$param coord_method:=$coord_method  score_run:=$score_run gazebo_obstacles:=$gazebo_obstacles p_task_initially_active:=$p_task_initially_active use_xbee:=$use_xbee end_time:=$end_time
pid="$pid $!"
sleep 1s
