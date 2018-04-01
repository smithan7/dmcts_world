#!/bin/bash

coord_method=$1

if [ $coord_method = 'g' ];
then
    coord_method="greedy_completion_reward"
fi


if [ $coord_method = 'm' ];
then
    coord_method="mcts_task_by_completion_reward_gradient"
fi

echo "coord_method = ${coord_method}"
sleep 5s

param=34
n_nodes=100
p_task_initially_active=0.4
n_agents=3
score_run=true
end_time=600.0
way_point_tol=1.0
use_hector_quad=true
world_display_map=true
agent_display_map=true
hardware_trial=false # more, is this a search and rescue mission
flat_tasks=false
speed_penalty=0.5
write_map_as_params=false
read_map_from_params=true

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
rosparam set "/package_directory" "$(rospack find dmcts_world)/"
rosparam set "/score_run" $score_run
rosparam set "/dmcts_world/display_map" $world_display_map
rosparam set "/agent_display_map" $agent_display_map
rosparam set "/use_gazebo" $use_gazebo
rosparam set "/hardware_trial" $hardware_trial
rosparam set "/flat_tasks" $flat_tasks
rosparam set "/speed_penalty" $speed_penalty
rosparam set "/n_task_types" 2
rosparam set "/n_agent_types" 2
rosparam set "/desired_altitude" ${zs[agent_index]}
rosparam set "/cruising_speed" ${cs[agent_index]}
rosparam set "/display_costmap_path" true
rosparam set "/read_map" $read_map_from_params
rosparam set "/write_map" $write_map_as_params
echo "Loaded ROS params"
sleep 2s

agent_index=$(rosparam get agent_index)

echo "Launching Simulated Quad"
roslaunch fake_dmcts_quad fake_dmcts_quad.launch agent_index:=$agent_index &
pid="$pid $!"
sleep 1s

echo "Launching Dist-MCTS Node"
roslaunch dmcts dmcts_dji.launch agent_index:=$agent_index &
pid="$pid $!" 
sleep 5s

echo "launching XBee for Agent"
roslaunch xbee_bridge xbee_bridge.launch &
pid="$pid $!"
sleep 1s

echo "initialiizing ROS-Bag"
rosbag record -a -O /home/andy/catkin_ws/bags_results/osu_field_${coord_method}_${agent_index}_${param_number}.bag