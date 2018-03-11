#!/bin/bash

param=$1
coord_method=$2

n_nodes=100
p_task_initially_active=0.4
n_agents=1
agent_index=0
score_run=true
gazebo_obstacles=false
use_gazebo=false
cruising_speed=1.0
use_xbee=false
end_time=120.0
way_point_tol=1.0
use_hector_quad=true
world_display_map=true
agent_display_map=true
hardware_trial=false # more, is this a search and rescue mission
flat_tasks=false
speed_penalty=0.5

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
sleep 2s

echo "Launching roscore..."
roscore &
pid="pid $!"
sleep 5s

# Declare starting locations and info for 6 agents
# effectively: low, high, low, high, low, high, low, high
declare -a xs=(-25.0 25.0 -25.0 025.0 25.0 0.00 000.0 -25.0)
declare -a ys=(-25.0 25.0 025.0 -25.0 00.0 25.0 -25.0 000.0)
declare -a zs=(005.0 20.0 007.5 022.5 10.0 25.0 010.0 027.5)
declare -a cs=(002.5 05.0 002.5 005.0 02.5 05.0 002.5 005.0)
declare -a agent_types=(0 1 0 1 0 1 0 1)
declare -a pay_obs_costs=(true false true false true false true false);

# Load specific stuff for this trial
echo "Loading ROS params"
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/osu_field_params.yaml"
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/willamette_park_params.yaml"
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/gazebo_map_params.yaml" &
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/dmcts_params.yaml"
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
rosparam set "/use_gazebo" $use_gazebo
rosparam set "/hardware_trial" $hardware_trial
rosparam set "/flat_tasks" $flat_tasks
rosparam set "/speed_penalty" $speed_penalty
rosparam set "/n_task_types" 2
rosparam set "/n_agent_types" 2
rosparam set "/agent_index" $agent_index
rosparam set "/desired_altitude" ${zs[agent_index]}
rosparam set "/cruising_speed" ${cs[agent_index]}
echo "Loaded ROS params"
sleep 2s


echo "Launching DJI-SDK"
roslaunch dji_sdk sdk_manifold.launch &
echo "Launched DJI-SDK"
sleep 5s

echo "Launching my pid controller"
    cd ~/catkin_ws
    ./src/my_quad_controller/scripts/dji_waypoint_controller.py &
    #.$(rospack find my_quad_controller)/scripts/dji_waypoint_controller.py &
echo "Launched pid controller"
sleep 5s

echo "Launching Dist-MCTS Node"
roslaunch dmcts dmcts_dji.launch agent_index:=$agent_index desired_altitude:=${zs[agent_index]} pay_obstacle_costs:=${pay_obs_costs[ai]} cruising_speed:=${cs[agent_index]} use_xbee:=$use_xbee use_hector_quad:=$use_hector_quad agent_type:=${agent_types[agent_index]}
pid="$pid $!" 
sleep 5s
