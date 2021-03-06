#!/bin/bash

n_agents=$1
n_nodes=$2
param=$3
coord_method=$4
score_run=true
p_task_initially_active=0.40
gazebo_obstacles=false
cruising_speed=2.0
use_xbee=false
end_time=100.0
 
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

echo "setting sim_time true"
#rosparam set /use_sim_time true &
#pid="$pid $!"
#sleep 1s


#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts fake quad nodes"
for ((ai=0; ai<n_agents; ai++))
do
    roslaunch fake_dmcts_quad fake_dmcts_quad.launch agent_index:=$ai agent_altitude:=${zs[ai]} start_x:=${xs[ai]} start_y:=${ys[ai]} cruising_speed:=$cruising_speed & 
    pid="$pid $!"
    sleep 1s
done

rosparam set /number_of_agents $n_agents
rosparam set /number_of_nodes $n_nodes
rosparam set /p_task_initially_active $p_task_initially_active
rosparam set /param_number $param
rosparam set /coord_method $coord_method
rosparam set /agent_index 0
rosparam set /test_environment_number $param
rosparam set /desired_altitude 5.0
rosparam set /pay_obstacle_costs false
rosparam set /cruising_speed $cruising_speed


echo "launching dmcts quad nodes"
for ((ai=0; ai<n_agents; ai++))
do
    if [ $ai -gt 0 ]
    then
        roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:=$coord_method  desired_altitude:=${zs[ai]} p_task_initially_active:=$p_task_initially_active pay_obstacle_costs:=${pay_obs_costs[ai]} not_simulation:=false cruising_speed:=$cruising_speed cruising_speed:=$cruising_speed param_number:=$param use_xbee:=$use_xbee &
        #roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='greedy_completion_reward' &
        #roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward' &
        #roslaunch dmcts dmcts_n.launch agent_index:=$ai num_agents:=$n_agents num_nodes:=$n_nodes coord_method:='mcts_task_by_completion_reward_impact_optimal' &
        pid="$pid $!"
        sleep 2s
    fi
done

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch num_agents:=$n_agents num_nodes:=$n_nodes param_number:=$param coord_method:=$coord_method  score_run:=$score_run gazebo_obstacles:=$gazebo_obstacles p_task_initially_active:=$p_task_initially_active use_xbee:=$use_xbee end_time:=$end_time
pid="$pid $!"
sleep 1s
#trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
#sleep 24h
