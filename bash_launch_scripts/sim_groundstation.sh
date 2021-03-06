#!/bin/bash

if [ $# -eq 0 ]
then
	while true
	do
		echo "************ No coordination method provided ******************"
		sleep 1s
	done
fi

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
sleep 3s

param=34
agent_index=-1
n_agents=3
n_nodes=100
p_task_initially_active=0.4
score_run=true
end_time=500.0
way_point_tol=5.0
use_hector_quad=false
world_display_map=true
agent_display_map=false
hardware_trial=false # more, is this a search and rescue mission
flat_tasks=false
speed_penalty=0.5
write_map_as_params=false
read_map_from_params=true
gazebo_obstacles=false
use_gazebo=false
use_xbee=false
record_bag=false
rand_num=$(date +%N | sed -e 's/000$//' -e 's/^0//')
 
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
sleep 5s

# Load specific stuff for this trial
echo "Loading ROS params"
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/osu_field_params.yaml"
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/willamette_park_params.yaml"
#rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/gazebo_map_params.yaml" &
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/dmcts_params.yaml"
rosparam load "$(rospack find dmcts_world)/bash_launch_scripts/launch_params/xbee_groundstation_params.yaml"
rosparam set "/agent_index" $agent_index
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
rosparam set "/read_map" $read_map_from_params
rosparam set "/write_map" $write_map_as_params
echo "Loaded ROS params"
sleep 2s

#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch &
pid="$pid $!"
sleep 1s


if $record_bag
then
	echo "initialiizing ROS-Bag"
	gnome-terminal -e 'bash -c "rosbag record -a -O ~/catkin_ws/bags_results/osu_field_'$coord_method'_'$agent_index'_'$param'_'$rand_num'.bag; exec bash"'
	pid="$pid $!"
	sleep 1s
fi


if $use_xbee
then

	echo "launching XBee for ground station"
	roslaunch xbee_bridge xbee_bridge.launch 
	pid="$pid $!"
	sleep 1s
fi

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h
