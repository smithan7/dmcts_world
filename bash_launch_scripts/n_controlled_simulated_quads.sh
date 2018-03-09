#!/bin/bash

n_agents=$1
n_nodes=$2
param=$3
coord_method=$4
p_task_initially_active=$5
score_run=true
gazebo_obstacles=false
cruising_speed=1.0
use_xbee=false
end_time=120.0
way_point_tol=5.0
use_hector_quad=false
world_display_map=true
agent_display_map=false

my_pid=$$
echo "My process ID is $my_pid"
echo "  n_agents: $n_agents"
echo "  n_nodes: $n_nodes"
echo "  param: $param"
echo "  coord_method: $coord_method"
echo "  p_task_initially_active: $p_task_initially_active"
echo "  min_sampling_threshold: $min_sampling_threshold"
echo "  way_point_tol: $way_point_tol"

echo "Sourcing ~/catkin_ws/devel/setup.bash"
source ~/catkin_ws/devel/setup.bash &
pid=$!

echo "Launching roscore..."
roscore &
pid="pid $!"
sleep 1s

# Declare starting locations and info for 6 agents
declare -a xs=(-25 25 -25 25 25 0 0 -25);
declare -a ys=(-25 25 25 -25 0 25 -25 0);
declare -a zs=(5 7.5 10 12.5 15 17.5 20 22.5);
declare -a cs=(5.0 5.0 5.0 5.0 5.0 5.0 5.0 5.0 5.0);
declare -a pay_obs_costs=(false false false false false false false false);

# Load specific stuff for this trial
echo "Loading ROS params"
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/osu_field_params.yaml"
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/park_params.yaml"
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/gazebo_map_params.yaml" &
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/dmcts_params.yaml"
rosparam set "/param_number" $param
rosparam set "/end_time" $end_time
rosparam set "/way_point_tol" $way_point_tol
rosparam set "/use_gazebo_obstacles" $gazebo_obstacles
rosparam set "/p_task_initially_active" $p_task_initially_active
rosparam set "/number_of_nodes" $n_nodes
rosparam set "/number_of_agents" $n_agents
rosparam set "/coord_method" $coord_method
rosparam set "/world_directory" "$(rospack find dmcts_world)/worlds/"
rosparam set "/score_run" $score_run
rosparam set "/dmcts_world/display_map" $world_display_map
rosparam set "/agent_display_map" $agent_display_map
echo "Loaded ROS params"
sleep 2s


echo "launching dmcts fake quad nodes"
for ((ai=0; ai<n_agents; ai++))
do
    roslaunch fake_dmcts_quad fake_dmcts_quad.launch agent_index:=$ai agent_altitude:=${zs[ai]} start_x:=${xs[ai]} start_y:=${ys[ai]} cruising_speed:=${cs[ai]} & 
    pid="$pid $!"
    sleep 1s
done

#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts quad nodes"
for ((ai=0; ai<n_agents; ai++))
do
    roslaunch dmcts dmcts_n.launch agent_index:=$ai desired_altitude:=${zs[ai]} pay_obstacle_costs:=${pay_obs_costs[ai]} cruising_speed:=${cs[ai]} use_xbee:=$use_xbee use_hector_quad:=$use_hector_quad &
    pid="$pid $!" 
    sleep 5s
done

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch use_xbee:=$use_xbee
pid="$pid $!"
sleep 1s
