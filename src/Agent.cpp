#include "Agent.h"
#include "World.h"
#include "Map_Node.h"
#include "Goal.h"
#include "Pose.h"

#include <iostream>
#include <ctime>


// ros stuff
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "angles/angles.h"
#include <custom_messages/Costmap_Bridge_Travel_Path_MSG.h>
#include "custom_messages/DMCTS_Task_List.h"
#include "custom_messages/DMCTS_Request_Task_List.h"
#include "custom_messages/DMCTS_Request_Work.h"
#include "custom_messages/DMCTS_Work_Status.h"
#include "custom_messages/DMCTS_Loc.h"


Agent::Agent(ros::NodeHandle nHandle, const int &index, const int &type, const double &travel_vel, const cv::Scalar &color, const bool &pay_obstacle_cost, const double &work_radius, World* world_in){
	this->initialized = false;
	this->world = world_in;
	this->edge.x = -1;
	this->edge.y = -1;
	this->edge_progress = 1.0;
	this->index = index;
	this->collected_reward = 0.0;
	this->work_radius = work_radius;
	this->type = type;
	this->travel_vel = travel_vel;
	this->travel_step = travel_vel * world->get_dt();
	this->pay_obstacle_cost = pay_obstacle_cost;
	this->color = color;
	this->n_tasks =  this->world->get_n_nodes();
	this->goal_node = new Goal();
	this->pose = new Pose(-1,-1,-1,0);
	this->m_node_initialized = false;
	this->initialized = true;
}

bool Agent::at_node(int node) {
	if (this->edge_progress >= 1.0 && this->edge.y == node) {
		return true;
	}
	else if (this->edge_progress == 0.0 && this->edge.x == node) {
		return true;
	}
	else {
		return false;
	}
}

cv::Point2d Agent::get_loc2d() {
	cv::Point2d p(0.0, 0.0);
	p.x = ( this->world->get_nodes()[this->edge.y]->get_x() -  this->world->get_nodes()[this->edge.x]->get_x())*this->edge_progress +  this->world->get_nodes()[this->edge.x]->get_x();
	p.y = ( this->world->get_nodes()[this->edge.y]->get_y() -  this->world->get_nodes()[this->edge.x]->get_y())*this->edge_progress +  this->world->get_nodes()[this->edge.x]->get_y();
	return p;
}
bool Agent::at_node() { // am I at a node, by edge progress?
	if (this->edge_progress == 0.0 || this->edge_progress >= 1.0) {
		return true;
	}
	else {
		return false;
	}
}

bool Agent::at_goal() { // am I at my goal node?
	if (this->edge_progress >= 1.0 && this->edge.y == this->goal_node->get_index()) {
		return true;
	}
	else if (this->edge_progress == 0.0 && this->edge.x == this->goal_node->get_index()) {
		return true;
	}
	else{
		return false;
	}
}

void Agent::update_pose(const double &xi, const double &yi, const double &zi, const double wi){
	this->pose->update_pose(xi, yi, zi, wi);
}

void Agent::update_edge(const int &ex, const int &ey){
	this->edge.x = ex;
	this->edge.y = ey;
}

Agent::~Agent(){
	delete this->goal_node;
	delete this->pose;
}
