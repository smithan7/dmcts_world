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
	
	World world = World(nHandle);
	ROS_INFO("World::world initialized");
	// return the control to ROS
	ros::spin();

	return 0;
}
