#!/bin/bash

param=34
coord_method=greedy_completion_reward

n_nodes=100
p_task_initially_active=0.1
n_agents=1
agent_index=0
score_run=true
gazebo_obstacles=false
use_gazebo=false
cruising_speed=1.0
use_xbee=false
end_time=12000.0
way_point_tol=1.0
use_hector_quad=true
world_display_map=true
agent_display_map=true
hardware_trial=false # more, is this a search and rescue mission
flat_tasks=false
speed_penalty=0.1

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

# Load specific stuff for this trial
echo "Loading ROS params"
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/osu_field_params.yaml"
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/willamette_park_params.yaml"
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/gazebo_map_params.yaml" &
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/${DJI_NAME}_params.yaml"
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/dmcts_params.yaml"
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/xbee_agent_params.yaml"
rosparam set "/param_number" $param
rosparam set "/end_time" $end_time
rosparam set "/way_point_tol" $way_point_tol
rosparam set "/use_gazebo_obstacles" $gazebo_obstacles
rosparam set "/p_task_initially_active" $p_task_initially_active
rosparam set "/number_of_nodes" $n_nodes
rosparam set "/number_of_agents" $n_agents
rosparam set "/coord_method" $coord_method
rosparam set "/world_directory" "$(rospack find dmcts_world)/"
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
rosparam set "/display_costmap_path" true
echo "Loaded ROS params"
sleep 2s


echo "Launching DJI-SDK"
roslaunch dji_sdk sdk_manifold.launch &
echo "Launched DJI-SDK"
sleep 5s

echo "Launching my pid controller"
    cd ~/catkin_ws
    ./src/my_quad_controller/scripts/dji_waypoint_controller.py &
echo "Launched pid controller"
sleep 5s

echo "Launching Dist-MCTS Node"
roslaunch dmcts dmcts_dji.launch agent_index:=$agent_index &
pid="$pid $!" 
sleep 5s

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch
pid="$pid $!"
sleep 1s

#echo "launching XBee for Agent"
#roslaunch xbee_bridge xbee_bridge.launch
#pid="$pid $!"
#sleep 1s
