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

echo "launching hector quadrotor with kinect"
roslaunch hector_quadrotor_gazebo spawn_quadrotor.launch &
pid="$pid $!"
sleep 5s

echo "Launching Gazebo..."
roslaunch gazebo_ros empty_world.launch & #willowgarage_world.launch &
pid="$pid $!"
sleep 5s

#echo "launching rviz"
#rviz &
#pid="$pid $!"
#sleep 5s

#echo "launching master node"
#python ~/catkin_ws/src/dmcts_python_scripts/m_node.py &
#pid="$pid $!"
#sleep 1s

echo "launching dmcts_node"
roslaunch dmcts_world dmcts_world.launch &
pi="$pid $!"
sleep 5s

echo "launching pid controller"
roslaunch my_quad_controller pid_controller.launch &
pid="$pid $!"
sleep 1s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 24h