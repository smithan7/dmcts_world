#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>

#include "World.h"

int main(int argc, char *argv[]){
	// initialization
	ros::init(argc, argv, "DMCTS_World");
	ros::NodeHandle nHandle("~");

	int test_environment_number = 0;
	int number_of_agents = 1;
	int params = 0;
	bool display_map = true;
	bool score_run = false;
	int n_nodes = 1;
	bool uga = false;
	double p_task_initially_active;

	std::string task_selection_method;
	//task_selection_method.append("greedy_completion_reward");
	std::string world_directory, map_name;

	ros::param::get("test_environment_number", test_environment_number);
	ros::param::get("number_of_agents", number_of_agents);
	ros::param::get("param_number", params);
	ros::param::get("world_directory", world_directory);
	ros::param::get("score_run", score_run);
	ros::param::get("display_map", display_map);
	ros::param::get("coord_method", task_selection_method);
	ros::param::get("number_of_nodes", n_nodes);
	ros::param::get("use_gazebo_obstacles", uga);
	ros::param::get("p_task_initially_active", p_task_initially_active);

	ROS_INFO("World::initializing world");
	ROS_INFO("   test_environment_number %i", test_environment_number);
	ROS_INFO("   number_of_agents %i", number_of_agents);
	ROS_INFO("   param_number %i", params);
	ROS_INFO("   world directory %s", world_directory.c_str());
	ROS_INFO("   score_run %i", score_run);
	ROS_INFO("   display_map %i", display_map);
	ROS_INFO("   coord_method %s", task_selection_method.c_str());
	ROS_INFO("   n_nodes %i", n_nodes);
	ROS_INFO("   use_gazebo_obstacles %i", uga);
	ROS_INFO("   p_task_initially_active %0.4f", p_task_initially_active);
	
	World world = World(nHandle, params, display_map, score_run, task_selection_method, world_directory, number_of_agents, n_nodes, uga, p_task_initially_active);
	ROS_INFO("World::world initialized");
	// return the control to ROS
	ros::spin();

	return 0;
}
