#pragma once



// ros stuff
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <vector>
// opencv stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>



class World;
class Agent_Planning;
class Agent_Coordinator;
class Goal;
class Pose;

class Agent
{
public:

	// functions
	Agent(ros::NodeHandle nHandle, const int &index, const int &type, const double &travel_vel, const cv::Scalar &color, const bool &pay_obstacle_cost, const double &work_radius, World* world_in);
	bool at_node(int node);
	~Agent();

	// access private variables
	Pose* get_pose() { return this->pose; };
	void set_pose(Pose* pose_in) {this->pose = pose_in; };
	void update_pose(const double &xi, const double &yi, const double &zi, const double wi);
	void update_edge(const int &ex, const int &ey);
	bool initialized, task_list_initialized, act_initialized, plan_initialized, m_node_initialized;

	int get_index() { return this->index; };
	cv::Point2d get_loc2d();
	int get_loc() { return this->edge.x; };

	Goal* get_goal() { return this->goal_node; };
	Agent_Coordinator* get_coordinator() { return this->coordinator; };
	Agent_Planning* get_planner() { return this->planner; };
	World* get_world() { return this->world; };
	double get_arrival_time() { return this->arrival_time; }
	
	int get_type() { return this->type; };
	double get_travel_vel() { return this->travel_vel; };
	double get_travel_step() { return this->travel_step; };
	bool get_pay_obstacle_cost() { return this->pay_obstacle_cost; };

	bool get_at_node() { return this->at_node(); };
	cv::Scalar get_color() { return this->color; };
	cv::Point2i get_edge() { return this->edge; };
	std::string get_task_selection_method() { return this->task_selection_method; };
	std::string get_task_claim_method() { return this->task_claim_method; };
	std::string get_task_claim_time() { return this->task_claim_time; };

	double get_work_done() { return this->work_done; };
	double get_travel_done() { return this->travel_done; };
	double get_collected_reward() { return this->collected_reward;};
	void set_collected_reward(const double &col_rew_in) { this->collected_reward += col_rew_in; };
	
	double get_edge_progress() { return this->edge_progress; };
	int get_edge_x() { return this->edge.x; };
	int get_edge_y() { return this->edge.y; };

private:
	// my location
	Pose* pose;

	// planning and coordinator
	Goal* goal_node;
	Agent_Planning* planner;
	Agent_Coordinator* coordinator;
	World* world;

	std::string task_selection_method; // how do I select tasks
	std::string task_claim_method; // how / when do I claim tasks
	std::string task_claim_time; // when do I claim my task
	
	double arrival_time; // for travel and arrival plan
	double expected_value;
	double work_radius; // how close can I be and still do work?

	double work_done; // accumulated reward
	double travel_done; // distance I have travelled
	double collected_reward; // how much reward I have collected

	cv::Point2i edge; // x:=where am I? y:=where am I going?
	double edge_progress; // how far along the edge am I?
	
	int index; // who am I in the world?
	int type; // what type of agent am I?
	double travel_vel; // how fast can I move?
	double travel_step; // how far do I move in one time step
	bool pay_obstacle_cost; // do obstacles affect me?
	cv::Scalar color; // what color am I plotted?
	int n_tasks; // how many tasks are there

	// functions
	bool at_node(); // am I at a node, by edge_progress
	bool at_goal(); // am I at my goal node?
	
	// working and planning
	void work_on_task(); // work on the task I am at
	
	// moving and path planning 
	void move_along_edge(); // move along the current edge
	void select_next_edge(); // have a goal, select next edge to get to goal
};

