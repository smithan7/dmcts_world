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
	World* get_world() { return this->world; };
	
	int get_type() { return this->type; };
	double get_travel_vel() { return this->travel_vel; };
	double get_travel_step() { return this->travel_step; };
	bool get_pay_obstacle_cost() { return this->pay_obstacle_cost; };

	bool get_at_node() { return this->at_node(); };
	cv::Scalar get_color() { return this->color; };
	cv::Point2i get_edge() { return this->edge; };

	double get_work_done() { return this->work_done; };
	double get_travel_done() { return this->travel_done; };
	double get_collected_reward() { return this->collected_reward;};
	void set_collected_reward(const double &col_rew_in) { this->collected_reward += col_rew_in; };
	
	double get_edge_progress() { return this->edge_progress; };
	int get_edge_x() { return this->edge.x; };
	int get_edge_y() { return this->edge.y; };

	void set_path(const std::vector<int> &path_in) {this->path = path_in; };
	std::vector<int> get_path() {return this->path; };

private:
	// my location
	Pose* pose;

	// What I am likely to do
	std::vector<int> path;

	// planning and coordinator
	Goal* goal_node;
	World* world;

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
};

