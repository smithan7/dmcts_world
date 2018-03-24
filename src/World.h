#pragma once

#include <ros/ros.h>
#include <custom_messages/Costmap_Bridge_Travel_Path_MSG.h>
#include "custom_messages/DMCTS_Coordination.h"
#include "custom_messages/DMCTS_Request_Task_List.h"
#include "custom_messages/DMCTS_Request_Work.h"
#include "custom_messages/DMCTS_Task_List.h"
#include "custom_messages/DMCTS_Work_Status.h"
#include "custom_messages/DMCTS_Loc.h"
#include "custom_messages/DMCTS_Pulse.h"
#include "gazebo_msgs/SpawnModel.h"
#include "rosgraph_msgs/Clock.h"

#include <vector>
// opencv stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class Map_Node;
class Agent;
class Task;

class World
{
public:
	World(ros::NodeHandle nHandle);
	
	~World();
	bool initialized;

	// ros stuff
	ros::Timer plot_timer, pulse_timer;
	ros::Duration plot_duration, pulse_duration;
	double start_time;
	bool initialized_clock;
	void plot_timer_callback(const ros::TimerEvent &e);
	void activate_task(const int &ti); // call from agent to activate task
	void deactivate_task(const int &ti); // deactivates task

    // Internal Subscriber
	ros::Timer clock_timer;
	ros::Duration clock_duration;
	void clock_timer_callback(const ros::TimerEvent &e);

    // Publish to Agents
    ros::Publisher task_list_pub, pulse_pub, work_status_pub;
    void publish_task_list();
	void pulse_timer_callback(const ros::TimerEvent &e);
	void publish_work_status(const int &n_index, const int &a_index, const int &node_status);

    // Subscribe to Agent topics
	ros::Subscriber request_task_list_sub, request_work_sub, loc_sub;
	void request_task_list_callback(const custom_messages::DMCTS_Request_Task_List &msg);
	void request_work_callback(const custom_messages::DMCTS_Request_Work &msg);
	void loc_callback(const custom_messages::DMCTS_Loc &msg);

	void spawn_gazebo_model();
	void delete_gazebo_node_model(const int &i);

	// accessing private vars
	std::vector<Agent*> get_agents() { return this->agents; };
	std::vector<Map_Node*> get_nodes() { return this->nodes; };
	int get_n_nodes() { return this->n_nodes; };
	int get_n_agents() { return this->n_agents; };
	double get_c_time() { return this->c_time; };
	double get_dt() { return this->dt; };
	double get_end_time() { return this->end_time; };
	cv::String get_task_selection_method() { return this->task_selection_method; };
	void get_task_status_list(std::vector<bool> &task_status, std::vector<int> &task_set);
	std::string get_mcts_search_type() { return this->mcts_search_type; };
	std::string get_mcts_reward_type() { return this->mcts_reward_type; };
	void set_mcts_reward_type(const std::string & rt) { this->mcts_reward_type = rt; };
	bool get_task_status(const int &ti) { return this->task_status_list[ti]; };
	int get_mcts_n_kids() { return this->mcts_n_kids; };
	double get_max_task_reward(const int &tt){ return this->max_task_rewards[tt]; };
	double get_min_task_reward(const int &tt){ return this->min_task_rewards[tt]; };
	double get_max_task_time(const int &tt){ return this->max_task_times[tt]; };
	double get_min_task_time(const int &tt){ return this->min_task_times[tt]; };
	double get_max_task_work(const int &tt){ return this->max_task_works[tt]; };
	double get_min_task_work(const int &tt){ return this->min_task_works[tt]; };

	// utility functions
	bool a_star(const int & start, const int & goal, const bool &pay_obstacle_cost, std::vector<int>& path, double & length);
	double dist2d(const double & x1, const double & x2, const double & y1, const double & y2);
	bool dist_between_nodes(const int & n1, const int & n2, double & d);
	void display_world(const int & ms);
	bool get_edge_cost(const int &n1, const int &n2, const bool &pay_obstacle_cost, double &cost);
	double get_height() { return double(this->map_height_meters); };
	double get_width() { return double(this->map_width_meters); };
	bool get_index(const std::vector<int>& vals, const int & key, int & index);
	bool get_mindex(const std::vector<double> &vals, int &mindex, double &minval);
	bool get_travel_time(const int & s, const int & g, const double & step_dist, const bool &pay_obstacle_cost, double & time);
	bool get_task_completion_time(const int &agent_index, const int &task_index, double &time);
	bool get_travel_cost(const int &s, const int &g, const bool &pay_obstacle_cost, double &cost);
	double rand_double_in_range(const double & min, const double & max);
	bool valid_node(const int & n);
	bool valid_agent(const int a);
	bool are_nbrs(const int &t1, const int &t2);
	int get_n_active_tasks();

private:
    void write_nodes_as_params();
    std::string map_name;
    bool read_map, write_map;
	double north_lat, south_lat, east_lon, west_lon;
	bool use_gazebo_obstacles, use_gazebo, hardware_trial;
    std::string test_environment_img, test_obstacle_img;
	std::vector<double> max_task_rewards, min_task_rewards;
	std::vector<double> max_task_times, min_task_times;
	std::vector<double> max_task_works, min_task_works;
	std::vector<double> starting_xs, starting_ys;
	cv::Mat travel_distances, obstacle_distances;
		
	int my_agent_index;
	std::vector<int> agent_status;
	double c_time, dt, end_time;
	std::string mcts_search_type, mcts_reward_type;
	int mcts_n_kids;
	bool show_display, score_run;
	double last_plot_time;
	int n_nodes, n_agents;
	int n_agent_types, n_task_types;
	bool flat_tasks;
	double p_task_initially_active, p_impossible_task, p_activate_task;
	double min_task_time, max_task_time, min_task_work, max_task_work, min_task_reward, max_task_reward;
	double min_travel_vel, max_travel_vel, min_agent_work, max_agent_work;
	double map_width_meters, map_height_meters, map_width_cells, map_height_cells;
	double p_pay_obstacle_cost; // probability that a generated agent will have to pay obstacle tolls

	int k_map_connections; // minimum number of connections in graph
	double k_connection_radius; // how close should I be to connect
	double p_connect; // probability of connecting if in radius
	double p_obstacle_on_edge; // probability an obstacle is on the edge
	double p_blocked_edge; // probability that an edge is blocked
	
	cv::Mat PRM_Mat;
	std::vector<Map_Node*> nodes;
	std::vector<Agent*> agents;
	std::vector<bool> task_status_list;

	cv::Mat Obs_Mat, Env_Mat;
	int n_obstacles;
	std::vector< std::vector<double> > obstacles;
	double obstacle_increase;
	double find_obstacle_costs(const int &i, const int &j, const double &free_dist);
	void create_obs_mat();
	void get_obs_mat();
	int inflation_iters;
	double inflation_box_size;
	double node_obstacle_threshold, inflation_sigma;
	double cells_per_meter, meters_per_cell;

	// record keeper
	std::vector<double> reward_captured;
	std::vector<double> reward_time;
	void record_work(const double &reward, const int &a_index);
	
	// initialize everything
	int param_file_index;
	std::string world_directory, package_directory;
	std::string task_selection_method;
	int rand_seed;
	char* param_file;
	void write_params();
	void load_params();
	void initialize_nodes_and_tasks();
	void read_nodes_and_tasks();
	void initialize_PRM();
	void generate_tasks();
	void initialize_agents(ros::NodeHandle nHandle);
	double get_global_distance(const double &lata, const double &lona, const double &latb, const double &lonb);
	double to_radians(const double &deg);
	void draw_prm_on_obs_mat();
	// do stuff
	void iterate_my_agent();
	void run_simulation();
	void clean_up_from_sim();
};


/*
cv::Mat obstacle_mat;
void create_random_obstacles();
void find_obstacle_costs();
*/

