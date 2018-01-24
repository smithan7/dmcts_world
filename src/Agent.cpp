#include "Agent.h"
#include "World.h"
#include "Map_Node.h"
#include "Agent_Coordinator.h"
#include "Agent_Planning.h"
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
#include "custom_messages/Get_Task_List.h"
#include "custom_messages/Complete_Work.h"
#include "custom_messages/Recieve_Agent_Locs.h"


Agent::Agent(ros::NodeHandle nHandle, const int &index, const int &type, const double &travel_vel, const cv::Scalar &color, const bool &pay_obstacle_cost, const double &work_radius, World* world_in){
	this->initialized = false;
	this->world = world_in;
	////////////////////// How do I select my goal ////////////////////////////////////////////
	{
		//////////////////////////////// greedy methods
		{
			//this->task_selection_method = "greedy_arrival_time"; // choose the closest active task
			//this->task_selection_method = "greedy_completion_time"; // choose the task I can finish first
			
			//this->task_selection_method = "greedy_current_reward"; // choose the task with the largest reward currently
			//this->task_selection_method = "greedy_arrival_reward"; // choose the task with the largest reward at the time I will arrive
			//this->task_selection_method = "greedy_completion_reward"; // choose the task with the largest reward at the time I will complete
		}
		//////////////////////////////// value methods
		{
			//this->task_selection_method = "value_current"; // choose task by value now, value = reward(t_current) - (travel_time + work_time)
			//this->task_selection_method = "value_arrival"; // choose task by value at time of arrival, value = reward(t_arrival) - (travel_time + work_time)
			//this->task_selection_method = "value_completion"; // choose task by value at time of completion, value = reward(t_complete) - (travel_time + work_time)
		}
		//////////////////////////////// impact methods
		{
			//this->task_selection_method = "impact_completion_reward"; // choose task by impact reward at time of completion, impact_reward = reward(t_complete) - reward(t^{next closest agent}_complete)
			//this->task_selection_method = "impact_completion_value"; // choose task by impact at value time of completion, impact_value = reward(t_complete) - reward(t^{next closest agent}_complete) - (travel_time + work_time)
		}


		//////////////////////////////// random methods
		{
			//this->task_selection_method = "random_nbr"; // choose a random nbr
			//this->task_selection_method = "random_node"; // choose a random node on the map
			//this->task_selection_method = "random_task"; // choose a random active task
		}
		///////////////////////////////// MCTS methods
		{
			//this->task_selection_method = "MCTS_value"; // use MCTS to plan a sequence of values 
		}
		this->task_selection_method =  this->world->get_task_selection_method();
	}
	//////////////////////// When do I claim my tasks /////////////////////////////////////////
	{
		this->task_claim_time = "completion_time"; // when I will complete the task
		//this->task_claim_time = "arrival_time"; // when I will arrive at the task
		//this->task_claim_time = "immediate"; // I claim it from right now
		//this->task_claim_time = "none"; // I do NOT claim the task at all
	}
	/////////////////////// How do I claim my tasks ///////////////////////////////////////////
	{
		this->task_claim_method = "greedy"; // whatever task is best gets P(t) = 1.0, else P(t) = 0.0;
		//this->task_claim_method = "sample"; // all tasks get P(t) = (V(t)-V_min)/(V_max-V_min);
	}

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
	this->planner = new Agent_Planning(this, world);
	this->coordinator = new Agent_Coordinator(this, n_tasks);
	this->pose = new Pose(-1,-1,-1,0);
	this->task_list_initialized = false;
	this->m_node_initialized = false;
	this->plan_initialized = false;
	this->act_initialized = false;
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
	delete this->planner;
	delete this->coordinator;
	delete this->goal_node;
	delete this->pose;
}