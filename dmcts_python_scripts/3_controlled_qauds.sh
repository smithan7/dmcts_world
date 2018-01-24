#!/bin/bash
 
my_pid=$$
echo "My process ID is $my_pid"

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

echo "launching hector quadrotor with kinect"
roslaunch hector_quadrotor_gazebo spawn_three_quadrotors.launch &
pid="$pid $!"
sleep 10s

#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 10s

echo "launching dmcts_world_node"
roslaunch dmcts_world dmcts_world.launch num_agents:=3 num_nodes:=50 &
pid="$pid $!"
sleep 10s

echo "launching quad_controllers 3"
roslaunch dmcts dmcts_3.launch num_agents:=3 num_nodes:=50 &
pid="$pid $!"
sleep 3s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h