#include "World.h"

#include "Map_Node.h"
#include "Agent.h"
#include "Goal.h"
#include "Pose.h"

#include <random>
#include <iostream>
#include <fstream>
#include <math.h>

// opencv stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <urdf/model.h>
#include <tf/transform_listener.h>

World::World(ros::NodeHandle nHandle, const int &param_file, const bool &display_plot, const bool &score_run, const std::string &task_selection_method, const std::string &world_directory, const int &number_of_agents_in, const int &n_nodes_in, const bool &use_gazebo_obstacles, const double &p_initially_active ) {
	this->initialized = false;
	this->initialized_clock = false;
	this->show_display = display_plot;
	this->score_run = score_run;
	this->param_file_index = param_file;
	this->task_selection_method = task_selection_method;
	this->mcts_search_type = "SW-UCT"; // UCT or SW-UCT
	this->mcts_reward_type = "normal"; // "impact";
	this->impact_style = "nn";
	this->mcts_n_kids = 10;
	this->world_directory = world_directory;
	this->n_agents = number_of_agents_in;
	this->flat_tasks = true;
	this->start_time = -1.0;
	this->p_task_initially_active = p_initially_active; // how likely is it that a task is initially active, 3-0.25, 5-0.5, 7-0.75

	// how often do I plot
	this->plot_duration = ros::Duration(1.0); 
	this->plot_timer = nHandle.createTimer(this->plot_duration, &World::plot_timer_callback, this);
	
	// how often do I just say that I am ok
	this->pulse_duration = ros::Duration(1.0);
	this->pulse_timer = nHandle.createTimer(this->pulse_duration, &World::pulse_timer_callback, this);

    // Internal Subscriber    
    this->clock_sub = nHandle.subscribe("/clock", 1, &World::clock_callback, this);
	
	// Subscribe to Agents
	this->request_task_list_sub = nHandle.subscribe("/dmcts_master/request_task_list", 10, &World::request_task_list_callback, this);
	this->loc_sub = nHandle.subscribe("/dmcts_master/loc", 10, &World::loc_callback, this);
	this->request_work_sub = nHandle.subscribe("/dmcts_master/request_work", 10, &World::request_work_callback, this);
	
	// Publishers to agents
	this->pulse_pub = nHandle.advertise<custom_messages::DMCTS_Pulse>("/dmcts_master/pulse", 10);
	this->task_list_pub = nHandle.advertise<custom_messages::DMCTS_Task_List>("/dmcts_master/task_list", 10);
	this->work_status_pub = nHandle.advertise<custom_messages::DMCTS_Work_Status>("/dmcts_master/work_status", 10);

	if (this->param_file_index > 0) {
		// load  everything from xml
		this->rand_seed = this->param_file_index;
		ROS_INFO("	Recieved RAND_SEED: %i", this->rand_seed);
		ROS_INFO("	Setting RAND_SEED param");
		ros::param::set("parameter_seed", this->rand_seed);
		//this->load_params();
	}
	else { // generate_params and then write them
		// seed the randomness in the simulator
		srand(int(time(NULL)));
		this->rand_seed = rand() % 1000000;
		ROS_INFO("	GENERATED RAND_SEED: %i", this->rand_seed);
		ROS_INFO("	Setting RAND_SEED param");
		ros::param::set("parameter_seed", this->rand_seed);
	}
	// time stuff
	this->c_time = 0.0;
	this->dt = 1.0;
	this->end_time = 10000.0;

	// map and PRM stuff
	this->map_height = 100.0; 
	this->map_width = 100.0; // 1000
	this->n_nodes = n_nodes_in; 
	this->k_map_connections = 5;
	this->k_connection_radius = 500.0;
	this->p_connect = 1.0;
	this->p_blocked_edge = 0.05;
	this->p_obstacle_on_edge = 0.2;
	this->p_pay_obstacle_cost = 0.0;
	
	// task stuff
	this->n_task_types = 4; // how many types of tasks are there
	this->p_impossible_task = 0.0; // how likely is it that an agent is created that cannot complete a task
	this->p_activate_task = 0.0;// 1.0*this->dt; // how likely is it that I will activate a task each second? *dt accounts per iters per second
	this->min_task_time = 1000.0; // shortest time to complete a task
	this->max_task_time = 6000.0; // longest time to complete a task
	this->min_task_work = 1.0; // min work to complete a task
	this->max_task_work = 1.0; // max work to complete a task
	this->min_task_reward = 100.0; // min reward for completing a task
	this->max_task_reward = 500.0; // max reward for completing a task

	// agent stuff
	// set through param this->n_agents = 2; // how many agents
	this->n_agent_types = 1; // how many types of agents
	this->min_travel_vel = 2.3; // 5 - slowest travel speed
	this->max_travel_vel = 2.7; // 25 - fastest travel speed
	this->min_agent_work = 100.0; // min amount of work an agent does per second
	this->max_agent_work = 100.0; // max amount of work an agent does per second

	// agent starting locations
	this->starting_locs.push_back(cv::Point2d(-25,-25));
	this->starting_locs.push_back(cv::Point2d(25,25));
	this->starting_locs.push_back(cv::Point2d(-25,25));
	this->starting_locs.push_back(cv::Point2d(25,-25));
	this->starting_locs.push_back(cv::Point2d(0,25));
	this->starting_locs.push_back(cv::Point2d(25,0));
	this->starting_locs.push_back(cv::Point2d(0,-25));
	this->starting_locs.push_back(cv::Point2d(-25,0));

	// reset randomization
	srand(this->rand_seed);

	// seed the obstacle mat
	this->make_obs_mat();
	
	// write params
	this->write_params();

	if (this->task_selection_method == "mcts_task_by_completion_reward_impact_before_and_after") {
		this->impact_style = "before_and_after";
		this->task_selection_method = "mcts_task_by_completion_reward_impact";
	}

	if (this->task_selection_method == "mcts_task_by_completion_reward_impact_optimal") {
		this->impact_style = "optimal";
		this->task_selection_method = "mcts_task_by_completion_reward_impact";
	}

	if (this->task_selection_method == "mcts_task_by_completion_reward_impact_fixed") {
		this->impact_style = "fixed";
		this->task_selection_method = "mcts_task_by_completion_reward_impact";
	}

	// reset randomization
	srand(this->rand_seed);

	// initialize map, tasks, and agents
	this->initialize_nodes_and_tasks();
	this->initialize_PRM();
	// initialize agents
	this->initialize_agents(nHandle);
	this->initialized = true;

	if(use_gazebo_obstacles){
		this->spawn_gazebo_model();
	}
}

void World::spawn_gazebo_model(){
	for(size_t i=0; i<this->obstacles.size(); i++){
		char stuff[200];
		int rr = floor(this->obstacles[i][2]);

		int n = sprintf(stuff, "rosrun gazebo_ros spawn_model -file /home/andy/catkin_ws/src/dmcts_world/urdf_models/cylinder_%i.urdf -urdf -model cyl_%i -y %0.2f -x %0.2f",rr, int(i), this->obstacles[i][1] - this->map_width/2, this->obstacles[i][0] - this->map_height/2);
		system(stuff);
		//system("rosrun gazebo_ros spawn_model -file /home/andy/catkin_ws/src/dmcts_world/urdf_models/cylinder_1.urdf -urdf -model cyl -y 2.2 -x -0.3");
	}
	for(size_t i=0; i<this->nodes.size(); i++){
		char stuff[200];
		int n = sprintf(stuff, "rosrun gazebo_ros spawn_model -file /home/andy/catkin_ws/src/dmcts_world/urdf_models/box.urdf -urdf -model box_%i -y %0.2f -x %0.2f", int(i), this->nodes[i]->get_y(), this->nodes[i]->get_x());
		system(stuff);
		//system("rosrun gazebo_ros spawn_model -file /home/andy/catkin_ws/src/dmcts_world/urdf_models/cylinder_1.urdf -urdf -model cyl -y 2.2 -x -0.3");
	}		
}

void World::delete_gazebo_node_model(const int &i){
		char stuff[200];
		int n = sprintf(stuff, "rosservice call gazebo/delete_model '{model_name: box_%i}", int(i));
		system(stuff);
		//rosservice call gazebo/delete_model '{model_name: coffee_cup}'
}

void World::make_obs_mat(){
	this->Obs_Mat = cv::Mat::zeros(this->map_width, this->map_height, CV_8UC1);
	this->obstacles.clear();
	ROS_INFO("DMCTS_World::World::make_obs_mat: making obstacles");
	while(this->obstacles.size() < 10){
		//ROS_INFO("making obstacle");
		// create a potnetial obstacle
		double rr = rand_double_in_range(1,10);
		double xx = rand_double_in_range(0,this->map_width);
		double yy = rand_double_in_range(0,this->map_height);
		//ROS_INFO("obs: %.1f, %.1f, r =  %.1f", xx, yy, rr);
		// check if any starting locations are in an obstacle
		bool flag = true;
		for(size_t s=0; s<this->starting_locs.size(); s++){
			double d = sqrt(pow(xx-(this->starting_locs[s].x+this->map_width/2),2) + pow(yy-this->starting_locs[s].y+this->map_height/2,2));
			//ROS_INFO("starting_locs: %.1f, %.1f, d = %.1f", this->starting_locs[s].x+this->map_width/2, this->starting_locs[s].y+this->map_height/2, d);
			if(rr+2 >= d ){
				// starting loc is in obstacle
				flag = false;
				break;
			}
		}

		if(flag){
			for(size_t s=0; s<this->obstacles.size(); s++){
				double d = sqrt(pow(xx-this->obstacles[s][0],2) + pow(yy-this->obstacles[s][1],2));
				if(rr+1 >= d || this->obstacles[s][2]+1 >= d){
					// obstacle is in obstacle
					flag = false;
					break;
				}
			}			
		}
		if(flag){
			std::vector<double> temp = {xx,yy,rr};
			this->obstacles.push_back(temp);
		}
	}

	for(int j=4; j>0; j--){
		for(size_t i=0; i<this->obstacles.size(); i++){
			cv::circle(this->Obs_Mat, cv::Point(this->obstacles[i][0], this->obstacles[i][1]), this->obstacles[i][2]+j, cv::Scalar(255 - 25*j), -1);	
		}
	}

	//cv::namedWindow("DMCTS_World::World::make_obs_mat:Obstacles", cv::WINDOW_NORMAL);
	//cv::imshow("DMCTS_World::World::make_obs_mat:Obstacles", this->Obs_Mat);
	//cv::waitKey(0);
}

void World::clock_callback(const rosgraph_msgs::Clock &tmIn){
	if(this->initialized_clock){
		this->c_time = tmIn.clock.now().toSec() - this->start_time;
		if(this->start_time < 0){
			this->start_time = tmIn.clock.now().toSec();
		}
	}
	else{
		this->c_time = 0.0;
	}
}

void World::request_task_list_callback(const custom_messages::DMCTS_Request_Task_List &msg){
	//ROS_ERROR("World::task_list_callback: recieved");
	
	custom_messages::DMCTS_Task_List bcst = custom_messages::DMCTS_Task_List();
	for(size_t i=0; i<this->nodes.size(); i++){
		if(this->nodes[i]->is_active()){
			bcst.node_indices.push_back(i);
		}
	}
	
	this->task_list_pub.publish(bcst);
	//ROS_ERROR("World::task_list_callback: assembled");
	//ROS_ERROR("World::task_list_callback: %i", int(resp.node_indices.size()));
	//ROS_ERROR("World::task_list_callback: %i", int(resp.reward.size()));
}

void World::request_work_callback(const custom_messages::DMCTS_Request_Work &msg){
	double work_done = 0.0;
	double reward_collected = 0.0;
	int complete = 0;
			
	try{
		if( this->nodes[msg.n_index] ){
		    if( this->nodes[msg.n_index]->is_active() ){
		        int a_type = this->agents[msg.a_index]->get_type();
		        this->nodes[msg.n_index]->get_worked_on(a_type, this->c_time, work_done, reward_collected);
			    this->reward_captured.push_back(reward_collected);
			    this->reward_time.push_back(this->c_time);

                if(this->nodes[msg.n_index]->is_active()){
                    if (work_done > 0.0){
                        complete = 1; // Succesfully did some work, but task is still active
                    }
                    else{
                        complete = -1; // Task is active and I did no work
                    }
                }
                else{
                    complete = 0; // Task is now complete
                }        
                
                this->publish_work_status(msg.n_index, msg.a_index, complete);


			    if(this->score_run){
			        this->record_work(reward_collected, msg.a_index);
			    }
		    }
		    else{
		        ROS_WARN("World::complete_work_callback: node[%i] is not valid", msg.n_index);
			    this->publish_work_status(msg.n_index, msg.a_index, -1);
			}
		}    
		else{
		    ROS_WARN("World::complete_work_callback: node[%i] is not active", msg.n_index);
            this->publish_work_status(msg.n_index, msg.a_index, -1);
		}
	}
	catch(...){
		ROS_ERROR("World::work_request_callback: node[%i] by agent %i failed", msg.n_index, msg.a_index);
	}
}

void World::publish_work_status(const int &n_index, const int &a_index, const int &node_status){
    custom_messages::DMCTS_Work_Status msg;
    msg.n_index = n_index;
    msg.a_index = a_index;
    msg.success = node_status;
    this->work_status_pub.publish(msg);
}

void World::record_work(const double &reward_captured, const int &agent_index){
    std::string rf;
    rf.append(this->world_directory);
    rf.append("results/");
    char temp[200];
    int n = sprintf(temp, "results_for_param_file_%i_%s.xml", this->rand_seed, this->task_selection_method.c_str());
    rf.append(temp);//this->param_file = temp_char;
    cv::FileStorage fs;
    fs.open(rf, cv::FileStorage::WRITE);
    //ROS_INFO("World::~World: writing results");
    //std::cout << "     " << rf << std::endl;
    // randomizing stuff in a controlled way
    fs << "task_selection_method" << this->task_selection_method;
    fs << "reward" << this->reward_captured;
    fs << "time" << this->c_time;
}

void World::loc_callback(const custom_messages::DMCTS_Loc &msg){
	//ROS_ERROR("World::odom recieved[%i]: %.2f, %.2f", msg.index, msg.xLoc, msg.yLoc);
	try {
		this->agents[msg.index]->update_pose(msg.xLoc, msg.yLoc, 0.0, 0.0);
		this->agents[msg.index]->update_edge(msg.edge_x, msg.edge_y);
		this->agent_status[msg.index] = msg.status;
		ROS_WARN("recieved locs: req.status[%i]: %i", msg.index, int(msg.status));
		if(!this->initialized_clock){
			// check if everyone should start and clock should be initialized
			bool flag = true;
			for(size_t i=0; i<this->agent_status.size(); i++){
				if(this->agent_status[i] == -1){
					flag = false;
					break;
				}
			}
			if(!this->initialized_clock && flag){
				this->start_time = -1;
				this->initialized_clock = true;
			}
		}
	}
	catch(...){
		ROS_ERROR("World::odom recieved but could not be updated[%i]: %.2f, %.2f", msg.index, msg.xLoc, msg.yLoc);
	}
}


void World::pulse_timer_callback(const ros::TimerEvent &e){
	custom_messages::DMCTS_Pulse msg;
	msg.my_index = -1;
	msg.c_time = this->c_time;
	msg.status = this->initialized_clock;
	msg.n_active_tasks = this->get_n_active_tasks();
	pulse_pub.publish(msg);
}

int World::get_n_active_tasks(){
	int cntr = 0;
	for(int i=0; i<this->n_nodes; i++){
		if(this->nodes[i]->is_active()){
			cntr++;
		}
	}
	return cntr;
}

void World::plot_timer_callback(const ros::TimerEvent &e){
	this->display_world(1);
}

bool World::are_nbrs(const int &t1, const int &t2) {
	if (t1 >= 0 && t1 < this->n_nodes) {
		if (this->nodes[t1]->is_nbr(t2)) {
			return true;
		}
	}
	return false;
}

void World::write_params() {

	if(!this->score_run){
		return;
	}

	std::string rf;
	rf.append(this->world_directory);
	rf.append("param_files/");
	char temp[200];
	int n = sprintf(temp, "param_file_%i.xml", this->rand_seed);
	rf.append(temp);//this->param_file = temp_char;
	cv::FileStorage fs;
	fs.open(rf, cv::FileStorage::WRITE);
	ROS_INFO("World::write_params: writing params");
	std::cout << "     " << rf << std::endl;

	// randomizing stuff in a controlled way
	fs << "rand_seed" << this->rand_seed;

	// time stuff
	fs << "c_time" << this->c_time;
	fs << "dt" << this->dt;
	fs << "end_time" << this->end_time;

	// map and PRM stuff
	fs << "map_height" << this->map_height;
	fs << "map_widht" << this->map_width;
	fs << "n_nodes" << this->n_nodes;
	fs << "k_map_connections" << this->k_map_connections;
	fs << "k_connection_radius" << this->k_connection_radius;
	fs << "p_connect" << this->p_connect; // probability of connecting
	fs << "p_obstacle_on_edge" << this->p_obstacle_on_edge;
	fs << "p_blocked_edge" << this->p_blocked_edge;
	fs << "p_pay_obstacle_cost" << this->p_pay_obstacle_cost;

	// task stuff
	fs << "n_task_types" << this->n_task_types; // how many types of tasks are there
	fs << "flat_tasks" << this->flat_tasks;
	fs << "p_task_initially_active" << this->p_task_initially_active; // how likely is it that a task is initially active
	fs << "p_impossible_task" << this->p_impossible_task; // how likely is it that an agent is created that cannot complete a task
	fs << "p_active_task" << this->p_activate_task; // how likely is it that I will activate a task each second? *dt accounts per iters per second
	fs << "min_task_time" << this->min_task_time; // shortest time to complete a task
	fs << "max_task_time" << this->max_task_time; // longest time to complete a task
	fs << "min_task_work" << this->min_task_work; // min work to complete a task
	fs << "max_task_work" << this->max_task_work; // max work to complete a task
	fs << "min_task_reward" << min_task_reward; // min reward for completing a task
	fs << "max_task_reward" << max_task_reward; // max reward for completing a task

   // agent stuff
	fs << "n_agents" << this->n_agents; // how many agents
	fs << "n_agent_types" << this->n_agent_types; // how many types of agents
	fs << "min_travel_vel" << this->min_travel_vel; // slowest travel speed
	fs << "max_travel_vel" << this->max_travel_vel; // fastest travel speed
	fs << "min_agent_work" << this->min_agent_work; // min amount of work an agent does per second
	fs << "max_agent_work" << this->max_agent_work; // max amount of work an agent does per second

	fs.release();
}

void World::load_params() {

	std::string rf;
	rf.append(this->world_directory);
	rf.append("param_files/");
	char temp[200];
	int n = sprintf(temp, "param_file_%i.xml", param_file_index);
	rf.append(temp);
	ROS_INFO("World::load_params: loading params");
	std::cout << "     " << rf << std::endl;
	cv::FileStorage fs;
	fs.open(rf, cv::FileStorage::READ);

	// rand seed stuff
	fs["rand_seed"] >> this->rand_seed;

	// time stuff
	fs["c_time"] >> this->c_time;
	fs["dt"] >> this->dt;
	fs["end_time"] >> this->end_time;

	// map and PRM stuff
	fs["map_height"] >> this->map_height;
	fs["map_widht"] >> this->map_width;
	fs["n_nodes"] >> this->n_nodes;
	fs["k_map_connections"] >> this->k_map_connections;
	fs["k_connection_radius"] >> this->k_connection_radius;
	fs["p_connect"] >> this->p_connect;
	fs["p_obstacle_on_edge"] >> this->p_obstacle_on_edge;
	fs["p_blocked_edge"] >> this->p_blocked_edge;
	fs["p_pay_obstacle_cost"] >> this->p_pay_obstacle_cost;

	// task stuff
	fs ["n_task_types"] >> this->n_task_types; // how many types of tasks are there
	fs ["flat_tasks"] >> this->flat_tasks; // are all tasks the same type and value
	fs ["p_task_initially_active"] >> this->p_task_initially_active; // how likely is it that a task is initially active
	fs ["p_impossible_task"] >> this->p_impossible_task; // how likely is it that an agent is created that cannot complete a task
	fs ["p_active_task"] >> this->p_activate_task; // how likely is it that I will activate a task each second? *dt accounts per iters per second
	fs ["min_task_time"] >> this->min_task_time; // shortest time to complete a task
	fs ["max_task_time"] >> this->max_task_time; // longest time to complete a task
	fs ["min_task_work"] >> this->min_task_work; // min work to complete a task
	fs ["max_task_work"] >> this->max_task_work; // max work to complete a task
	fs ["min_task_reward"] >> this->min_task_reward; // min reward for completing a task
	fs ["max_task_reward"] >> this->max_task_reward; // max reward for completing a task

   // agent stuff
	fs ["n_agents"] >> this->n_agents; // how many agents
	fs ["n_agent_types"] >> this->n_agent_types; // how many types of agents
	fs ["min_travel_vel"] >> this->min_travel_vel; // slowest travel speed
	fs ["max_travel_vel"] >> this->max_travel_vel; // fastest travel speed
	fs ["min_agent_work"] >> this->min_agent_work; // min amount of work an agent does per second
	fs ["max_agent_work"] >> this->max_agent_work; // max amount of work an agent does per second
	
	fs.release();


}

void World::generate_tasks() {
	// generate with certain probability each time step
	if (true) {
		double r = this->rand_double_in_range(0.0, 1.0);
		if (r < this->p_activate_task) {
			bool flag = true;
			int iter = 0;
			while (flag && iter < 100) {
				iter++;
				int c_task = rand() % this->n_nodes;
				if (!this->nodes[c_task]->is_active()) {
					this->nodes[c_task]->activate(this);
					flag = false;
				}
			}
		}
	}

	for (int n = 0; n < this->n_nodes; n++) {
		this->task_status_list[n] = this->nodes[n]->is_active();
	}
}

void World::activate_task(const int &ti){
	this->nodes[ti]->activate(this);
}

void World::deactivate_task(const int &ti){
	this->nodes[ti]->deactivate();
}

bool World::get_task_completion_time(const int &ai, const int &ti, double &time) {
	if (this->valid_agent(ai) && this->valid_node(ti)) {
		time = this->nodes[ti]->get_time_to_complete(this->agents[ai], this);
		return true;
	}
	else {
		return false;
	}
}

bool World::valid_agent(const int a) {
	if (a < this->n_agents && a > -1) {
		return true;
	}
	else {
		return false;
	}
}

bool World::get_travel_cost(const int &s, const int &g, const bool &pay_obstacle_cost, double &cost) {
	if (this->valid_node(s) && this->valid_node(g)) {
		std::vector<int> path;
		if (this->a_star(s, g, pay_obstacle_cost, path, cost)) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

bool World::get_travel_time(const int &s, const int &g, const double &step_dist, const bool &pay_obstacle_cost, double &time) {
	if (this->valid_node(s) && this->valid_node(g)) {
		double dist;
		std::vector<int> path;
		if (this->a_star(s, g, pay_obstacle_cost, path, dist)) {
			time = dist / step_dist;
			return true;
		}
		else {
			return false;
		}
	
	}
	else {
		return false;
	}
}

bool World::valid_node(const int &n) {
	if (n < this->n_nodes && n > -1) {
		return true;
	}
	else {
		return false;
	}
}

void World::get_task_status_list(std::vector<bool> &task_status_list, std::vector<int> &task_set) {
	task_set.clear();
	task_status_list.clear();

	task_status_list = this->task_status_list;
	for (size_t i = 0; i < this->task_status_list.size(); i++) {
		if (this->task_status_list[i]) {
			task_set.push_back(int(i));
		}
	}
}

void World::display_world(const int &ms) {

	if (!this->show_display){
		return;
	}
	if(!this->initialized) {
		ROS_WARN("World::display_world: Agent %i world not initialized", this->my_agent_index);
		return;
	}

	cv::Scalar red(0.0, 0.0, 255.0);
	cv::Scalar blue(255.0, 0.0, 0.0);
	cv::Scalar green(0.0, 255.0, 0.0);
	cv::Scalar white(255.0, 255.0, 255.0);
	cv::Scalar orange(69.0, 100.0, 255.0);
	cv::Scalar black(0.0, 0.0, 0.0);
	cv::Scalar gray(127.0, 127.0, 127.0);

	double des_x = 1000.0;
	double des_y = 1000.0;

	double scale_x = des_x / this->map_width;
	double scale_y = des_y / this->map_height;

	if (this->PRM_Mat.empty()) {
		this->PRM_Mat = cv::Mat::zeros(int(des_x), int(des_y), CV_8UC3);
		// draw obstacles
		for(size_t i=0; i<this->obstacles.size(); i++){
			cv::Point2d pp = cv::Point2d(this->obstacles[i][0]*scale_x, this->obstacles[i][1]*scale_y);
			cv::circle(this->PRM_Mat, pp, this->obstacles[i][2]*scale_x, cv::Scalar(127, 127, 127), -1);
		}

		// draw PRM connections
		for (int i = 0; i < this->n_nodes; i++) {
			int index = -1;
			for(int iter=0; iter<this->nodes[i]->get_n_nbrs(); iter++){
				if (this->nodes[i]->get_nbr_i(iter, index)) {
					double cost = 0.0;
					double dist = 0.0;
					if (this->nodes[i]->get_nbr_obstacle_cost(iter, cost) && this->nodes[i]->get_nbr_distance(iter, dist)) {
						cost = (cost - dist) / cost;
						cv::Vec3b pink(uchar(255.0*(1.0 - cost)), uchar(255.0*(1.0 - cost)), 255);
						cv::Point2d p1 = this->nodes[i]->get_loc();
						cv::Point2d p2 = this->nodes[index]->get_loc();
						p1.x = scale_x * (p1.x + this->map_width/2); p1.y = scale_y * (p1.y + this->map_height/2);
						p2.x = scale_x * (p2.x + this->map_width/2); p2.y = scale_y * (p2.y + this->map_height/2);
						cv::line(this->PRM_Mat, p1, p2, pink, 2);
					}
				}
			}
		}

		// draw nodes
		for (int i = 0; i < this->n_nodes; i++) {
			cv::Point2d p1 = this->nodes[i]->get_loc();
			p1.x = scale_x * (p1.x + this->map_width/2); p1.y = scale_y * (p1.y + this->map_height/2);
			cv::circle(this->PRM_Mat, p1, 5, cv::Scalar(255, 255, 255), -1);
		}

		// label tasks
		for (int i = 0; i < this->n_nodes; i++) {
			double d = -5.0;
			cv::Point2d p1 = this->nodes[i]->get_loc();
			p1.x = scale_x * (p1.x + this->map_width/2); p1.y = scale_y * (p1.y + this->map_height/2);
			cv::Point2d tl = cv::Point2d(p1.x - d, p1.y + d);
			char text[10];
			sprintf(text, "%i", i);
			if (this->nodes[i]->is_active()) {
				cv::putText(this->PRM_Mat, text, tl, CV_FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(255, 255, 255), 3);
			}
		}
	}

	cv::Mat temp = this->PRM_Mat.clone();
	cv::Mat map = cv::Mat::zeros(cv::Size(int(des_x), int(des_y) + 100), CV_8UC3);
	temp.copyTo(map(cv::Rect(0, 0, temp.cols, temp.rows)));

	
	// draw active tasks
	for (int i = 0; i < this->n_nodes; i++) {
		if (this->nodes[i]->is_active()) {
			double d = 15.0;
			cv::Point2d p1 = this->nodes[i]->get_loc();
			p1.x = scale_x * (p1.x + this->map_width/2); p1.y = scale_y * (p1.y + this->map_height/2);
			cv::Point2d tl = cv::Point2d(p1.x - d, p1.y + d);
			cv::Point2d br = cv::Point2d(p1.x + d, p1.y - d);
			cv::rectangle(map, cv::Rect(tl, br), this->nodes[i]->get_color(), -1);
		}
	}

	/*
	// draw agent goals
	for (int i = 0; i < this->n_agents; i++) {
		if(this->agents[i]->get_goal()){
			cv::Point2d l = this->nodes[this->agents[i]->get_goal()->get_index()]->get_loc();
			l.x *= scale_x; l.y *= scale_y;
			cv::circle(map, l, 8, red, -1);
			cv::circle(map, l, 4, black, -1);
		}
	}
	*/

	// draw agents
	for (int i = 0; i < this->n_agents; i++) {
		// draw their location
		cv::Point2d p1(this->agents[i]->get_pose()->get_x(), this->agents[i]->get_pose()->get_y());
		p1.x = scale_x * (p1.x + this->map_width/2); p1.y = scale_y * (p1.y + this->map_height/2);
		if(p1.x >= 0.0 && p1.x <= des_x && p1.y >= 0.0 && p1.y <= des_y){
			cv::circle(map, p1, 8, red, -1);
		
			// draw line to their edge x
			if(this->agents[i]->get_edge_x() >= 0 && this->agents[i]->get_edge_x() <= this->n_nodes){		
				cv::Point2d p2 = this->nodes[this->agents[i]->get_edge_x()]->get_loc();
				p2.x = scale_x * (p2.x + this->map_width/2); p2.y = scale_y * (p2.y + this->map_height/2);
				cv::line(map, p1, p2, red, 2);
			}
			
			// draw line to their edge y
			if(this->agents[i]->get_edge_y() >= 0 && this->agents[i]->get_edge_y() <= this->n_nodes){
				cv::Point2d p3 = this->nodes[this->agents[i]->get_edge_y()]->get_loc();
				p3.x = scale_x * (p3.x + this->map_width/2); p3.y = scale_y * (p3.y + this->map_height/2);
				cv::line(map, p1, p3, red, 2);
			}

			char text[10];
			sprintf(text, "%i", i);
			cv::putText(map, text, p1, CV_FONT_HERSHEY_COMPLEX, 1.0, red, 3);
		}
	}
	
	cv::putText(map, this->task_selection_method, cv::Point2d(40.0, des_y + 30.0), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);
	char time[200];
	sprintf(time, "Time: %.2f of %.2f", this->c_time, this->end_time);
	cv::putText(map, time, cv::Point2d(30.0, des_y + 80.0), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);
	char map_name[200];
	sprintf(map_name, "Map Name: %i", this->rand_seed);

	cv::putText(map, map_name, cv::Point2d(500.0, des_y + 80), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);
	
	double current_plot_time = 1000.0 * clock() / double(CLOCKS_PER_SEC);
	cv::namedWindow("Map World", cv::WINDOW_NORMAL);
	cv::imshow("Map World", map);
	if (ms == 0) {
		cv::waitKey(0);
	}
	else if (double(ms) - (this->last_plot_time - current_plot_time) < 0.0) {
		cv::waitKey(100);//ms - int(floor(this->last_plot_time - current_plot_time)));
	}
	else {
		cv::waitKey(100);
	}

	this->last_plot_time = 1000.0 * clock() / double(CLOCKS_PER_SEC);
}

void World::initialize_agents(ros::NodeHandle nHandle) {

	std::string rf;
	rf.append(this->world_directory);
	rf.append("/param_files/");
	char temp[200];
	int n = sprintf(temp, "param_file_%i.xml", param_file_index);
	rf.append(temp);
	cv::FileStorage fs;
	fs.open(rf, cv::FileStorage::READ);

	std::vector<double> agent_travel_vels;
	//fs["agent_travel_vels"] >> agent_travel_vels;
	std::vector<bool> agent_obstacle_costs;
	//fs["agent_obstacle_costs"] >> agent_obstacle_costs;
	std::vector<double> agent_work_radii;
	//fs["agent_work_radii"] >> agent_work_radii;
	std::vector<int> agent_types;
	//fs["agent_types"] >> agent_types;
	for(int i=0; i<this->n_agents; i++){
		agent_types.push_back(0);
	}

	std::vector<cv::Scalar> agent_colors;
	for (int i = 0; i < this->n_agent_types; i++) {
		agent_travel_vels.push_back(2.0);
		agent_obstacle_costs.push_back(true);
		agent_work_radii.push_back(5.0);
		if(this->my_agent_index == i){
			double r = 255.0;
			double b = 0.0;
			double g = 0.0;

			cv::Scalar color(b, g, r);
			agent_colors.push_back(color);	
		}
		else{
			double r = 0.0;
			double b = 0.0;
			double g = 0.0;

			cv::Scalar color(b, g, r);
			agent_colors.push_back(color);
		}
	}
	this->agent_status.clear();
	for (int i = 0; i < this->n_agents; i++) {
		this->agent_status.push_back(-1);
		int tp = agent_types[i];
		Agent* a = new Agent(nHandle, i, tp, agent_travel_vels[tp], agent_colors[tp], agent_obstacle_costs[tp], agent_work_radii[tp], this);
		this->agents.push_back(a);
	}
}

void World::initialize_PRM() {
	// get travel distance between all nodes
	this->travel_distances = cv::Mat(this->n_nodes, this->n_nodes, CV_32F, -1);
	this->obstacle_distances = cv::Mat(this->n_nodes, this->n_nodes, CV_32F, -1);


	// connect all nodes within radius
	this->task_status_list.clear();
	for (int i = 0; i < this->n_nodes; i++) {
		// all tasks are started off
		this->task_status_list.push_back(false);
		for (int j = i+1; j < this->n_nodes; j++) {
			double d;
			if (this->dist_between_nodes(i, j, d)) {
				if (d < this->k_connection_radius && rand() < this->p_connect) { // am I close enough?
					double obs_cost = d;
					
					if (this->rand_double_in_range(0.0, 1.0) < this->p_obstacle_on_edge) { // does travel type two pay obstacle cost?
						if (this->rand_double_in_range(0.0, 1.0) < this->p_blocked_edge) { // does travel type two get blocked from using edge
							obs_cost = double(INFINITY);
						}
						else { // pay obstacle costs
							obs_cost = d*(1 + this->rand_double_in_range(0.0, 1.0));
						}
					}
					// set normal nbr and travel
					this->nodes[i]->add_nbr(j, d, obs_cost);
					this->nodes[j]->add_nbr(i, d, obs_cost);
					this->travel_distances.at<float>(i,j) = d;
					this->obstacle_distances.at<float>(i,j) = obs_cost;
					this->travel_distances.at<float>(j,i) = d;
					this->obstacle_distances.at<float>(j,i) = obs_cost;
				}
			}
		}

		// if not enough connections were done, add some more connections until min is reached
		this->k_map_connections = std::min(this->k_map_connections, this->n_nodes-1);
		while(this->nodes[i]->get_n_nbrs() < this->k_map_connections){
			double min_dist = double(INFINITY);
			int mindex = -1;
			
			for (int j = 0; j < this->n_nodes; j++) {
				if (i != j) {
					bool in_set = false;
					for (int k = 0; k < this->nodes[i]->get_n_nbrs(); k++) {
						int n_i = 0;
						if (this->nodes[i]->get_nbr_i(k,n_i)){
							if (n_i == j) {
								in_set = true;
								break;
							}
						}
					}
					if (!in_set) {
						double d;
						if (this->dist_between_nodes(i, j, d)) {
							if (d < min_dist) {
								min_dist = d;
								mindex = j;
							}
						}
					}
				}
			}
			//cv::namedWindow("obstacles", cv::WINDOW_NORMAL);
			//cv::imshow("obstacles", this->Obs_Mat);
			//cv::waitKey(100);

			// get node[i]'s location on the img
			cv::Point me = this->nodes[i]->get_loc();
			me.x += this->map_width/2;
			me.y += this->map_height/2;
			// get their location
			cv::Point np = this->nodes[mindex]->get_loc();
			np.x += this->map_width/2;
			np.y += this->map_height/2;
			// get a line iterator from me to them
			cv::LineIterator lit(this->Obs_Mat, me, np);
			double val_sum = 0.0;
			// count every obstacles cell between me and them!
			for (int i = 0; i < lit.count; i++, ++lit) {
				// count along line
				val_sum += double(this->Obs_Mat.at<uchar>(lit.pos()))/255.0;
			}
			double mean_val = val_sum / double(lit.count);
			double obs_cost = min_dist*(1+mean_val);
			this->nodes[i]->add_nbr(mindex, min_dist, obs_cost);
			this->nodes[mindex]->add_nbr(i, min_dist, obs_cost);
			this->travel_distances.at<float>(i,mindex) = min_dist;
			this->obstacle_distances.at<float>(i,mindex) = obs_cost;
			this->travel_distances.at<float>(mindex,i) = min_dist;
			this->obstacle_distances.at<float>(mindex,i) = obs_cost;
			//ROS_INFO("%i -> %i: %0.1f", i, mindex, min_dist);
		}
	}

	/*
	ROS_ERROR("travel_distances");
	for(int i=0; i<this->n_nodes; i++){
		for(int j=0; j<this->n_nodes; j++){
			std::cout << this->obstacle_distances.at<float>(i,j) << ",";
		}
		std::cout << std::endl;
	}
	*/

	for(int i=0; i<this->n_nodes; i++){
		for(int j=i; j<this->n_nodes; j++){
			if(this->travel_distances.at<float>(i,j) == -1){
				double d = 0.0;
				std::vector<int> path;
				if(a_star(i, j, false, path, d)){
					this->travel_distances.at<float>(i,j) = float(d);
					this->travel_distances.at<float>(j,i) = float(d);
				}
				else{
					this->travel_distances.at<float>(i,j) = INFINITY;
					this->travel_distances.at<float>(j,i) = INFINITY;
				}
				if(a_star(i, j, true, path, d)){
					this->obstacle_distances.at<float>(i,j) = float(d);
					this->obstacle_distances.at<float>(j,i) = float(d);
				}
				else{
					this->obstacle_distances.at<float>(i,j) = INFINITY;
					this->obstacle_distances.at<float>(j,i) = INFINITY;
				}
			}
		}
	}

	/*
	ROS_ERROR("travel_distances prime");
	for(int i=0; i<this->n_nodes; i++){
		for(int j=0; j<this->n_nodes; j++){
			std::cout << this->obstacle_distances.at<float>(i,j) << ",";
		}
		std::cout << std::endl;
	}
	*/
}

double World::rand_double_in_range(const double &min, const double &max) {
	// get random double between min and max
	return (max - min) * double(rand()) / double(RAND_MAX) + min;
}

double World::dist2d(const double &x1, const double &x2, const double &y1, const double &y2) {
	// get distance between 1 and 2
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool World::get_edge_cost(const int &n1, const int &n2, const bool &pay_obstacle_cost, double &cost) {
	if (n1 >= 0 && n1 < this->n_nodes) {
		if (n1 == n2) {
			cost = 0.0;
			return true;
		}
		int nbr_index = -1;
		for (int ni = 0; ni < this->nodes[n1]->get_n_nbrs(); ni++) {
			if (this->nodes[n1]->get_nbr_i(ni, nbr_index)) {
				if (nbr_index == n2) {
					this->nodes[n1]->get_nbr_travel_cost(ni, pay_obstacle_cost, cost);
					return true;
				}
			}
		}
	}
	return false;
}

bool World::dist_between_nodes(const int &n1, const int &n2, double &d) {
	// am I a node?
	if (n1 >= 0 && n1 < this->n_nodes && n2 >= 0 && n2 < this->n_nodes) {
		// get distance between nodes
		d = sqrt(pow(this->nodes[n1]->get_x() - this->nodes[n2]->get_x(), 2) + pow(this->nodes[n1]->get_y() - this->nodes[n2]->get_y(), 2));
		return true;
	}
	else {
		return false;
	}
}

void World::initialize_nodes_and_tasks() {
	this->min_task_times.clear();
	this->max_task_times.clear();
	this->min_task_rewards.clear();
	this->max_task_rewards.clear();
	this->min_task_works.clear();
	this->max_task_works.clear();
	std::vector<cv::Scalar> task_colors;
	std::vector<std::vector<double> > task_work_by_agent;
	for (int t = 0; t < this->n_task_types; t++) {

		double r = rand_double_in_range(0.0, 255.0);
		double b = rand_double_in_range(0.0, 255.0);
		double g = rand_double_in_range(0.0, 255.0);

		cv::Scalar color(b, g, r);
		task_colors.push_back(color);

		double t0 = this->rand_double_in_range(this->min_task_time, this->max_task_time);
		double t1 = this->rand_double_in_range(this->min_task_time, this->max_task_time);
		this->min_task_times.push_back(std::min(t0,t1));
		this->max_task_times.push_back(std::max(t0,t1));

		t0 = this->rand_double_in_range(this->min_task_reward, this->max_task_reward);
		t1 = this->rand_double_in_range(this->min_task_reward, this->max_task_reward);
		this->min_task_rewards.push_back(std::min(t0,t1));
		this->max_task_rewards.push_back(std::max(t0,t1));

		t0 = this->rand_double_in_range(this->min_task_work, this->max_task_work);
		t1 = this->rand_double_in_range(this->min_task_work, this->max_task_work);
		this->min_task_works.push_back(std::min(t0,t1));
		this->max_task_works.push_back(std::max(t0,t1));

		// set task times
		std::vector<double> at;
		for (int a = 0; a < this->n_agent_types; a++) {
			if (this->rand_double_in_range(0.0, 1.0) < this->p_impossible_task) {
				at.push_back(double(INFINITY));
			}
			else {
				at.push_back(this->rand_double_in_range(this->min_agent_work, this->max_agent_work));
			}
		}
		task_work_by_agent.push_back(at);
	}

	double x,y;
	for(size_t i=0; i<this->starting_locs.size(); i++){
		int task_type = rand() % n_task_types;
		Map_Node* n = new Map_Node(this->starting_locs[i].x, this->starting_locs[i].y, i, this->p_task_initially_active, task_type, task_work_by_agent[task_type], task_colors[task_type], this->flat_tasks, this);
		this->nodes.push_back(n);
	}


	int i = int(this->nodes.size());
	while(i < this->n_nodes) {
		bool flag = true;

		while(flag){
			x = this->rand_double_in_range(-this->map_width/2, this->map_width/2);
			y = this->rand_double_in_range(-this->map_height/2, this->map_height/2);
			if(this->Obs_Mat.at<uchar>(cv::Point(x + this->map_width/2,y+this->map_height/2)) == 0){
				flag = false;
			}
		}
		int task_type = rand() % n_task_types;
		Map_Node* n = new Map_Node(x, y, i, this->p_task_initially_active, task_type, task_work_by_agent[task_type], task_colors[task_type], this->flat_tasks, this);
		this->nodes.push_back(n);
		i++;
	}
}

World::~World(){
	for (size_t i = 0; i < this->nodes.size(); i++) {
		delete this->nodes[i];
	}
	this->nodes.clear();

	for (size_t i = 0; i < this->agents.size(); i++) {
		delete this->agents[i];
	}
	this->agents.clear();
}

bool World::a_star(const int &start, const int &goal, const bool &pay_obstacle_cost, std::vector<int> &path, double &length) {

	if (start < 0 || start >= this->n_nodes) {
		ROS_ERROR("World::a_star:: start off graph");
		return false;
	}
	if (goal < 0 || goal >= this->n_nodes) {
		ROS_ERROR("World::a_star:: goal off graph");
		return false;
	}

	// garbage variables
	int trash_i = -1;
	double trash_d = -1.0;

	// The set of nodes already evaluated
	std::vector<int> closed_set;

	// The set of currently discovered nodes that are not evaluated yet.
	// Initially, only the start node is known.
	std::vector<int> open_set;
	open_set.push_back(start);

	// For each node, which node it can most efficiently be reached from.
	// If a node can be reached from many nodes, cameFrom will eventually contain the
	// most efficient previous step.
	std::vector<int> came_from(this->n_nodes, -1);

	// For each node, the cost of getting from the start node to that node.
	std::vector<double> gScore(this->n_nodes, double(INFINITY));

	// The cost of going from start to start is zero.
	gScore[start] = 0.0;

	// For each node, the total cost of getting from the start node to the goal
	// by passing by that node. That value is partly known, partly heuristic.
	std::vector<double> fScore(this->n_nodes, double(INFINITY));
	

	// For the first node, that value is completely heuristic.
	this->dist_between_nodes(start, goal, fScore[start]);

	while (open_set.size() > 0) {
		// the node in openSet having the lowest fScore[] value
		int current = -1;
		if (!get_mindex(fScore, current, trash_d)) {
			return false;
		}
		if (current == goal) {
			path.clear();
			length = gScore[current];
			path.push_back(current);
			while (current != start) {
				current = came_from[current];
				path.push_back(current);
			}
			return true;
		}

		int index = 0;
		if (this->get_index(open_set, current, index)) {
			fScore[current] = double(INFINITY);
			open_set.erase(open_set.begin() + index);
		}
		closed_set.push_back(current);

		int n_nbrs = this->nodes[current]->get_n_nbrs();
		for (int ni = 0; ni < n_nbrs; ni++) {
			int neighbor = -1;
			if (this->nodes[current]->get_nbr_i(ni, neighbor)) {
				if (this->get_index(closed_set, neighbor, trash_i)) { // returns false if not in set
					continue;	// Ignore the neighbor which is already evaluated.
				}

				if (!this->get_index(open_set, neighbor, trash_i)) {// Discover a new node
					open_set.push_back(neighbor);
				}

				// The distance from start to a neighbor
				//if (this->dist_between_nodes(current, neighbor, trash_d)) {
				if(this->nodes[current]->get_nbr_travel_cost(ni, pay_obstacle_cost, trash_d)){
					double tentative_gScore = gScore[current] + trash_d;
					if (tentative_gScore >= gScore[neighbor]) {
						continue;		// This is not a better path.
					}

					// This path is the best until now. Record it!
					came_from[neighbor] = current;
					gScore[neighbor] = tentative_gScore;
					if (this->dist_between_nodes(neighbor, goal, trash_d)) {
						fScore[neighbor] = gScore[neighbor] + trash_d;
					}
				}
			}
		}
	}
	return false;
}

bool World::get_mindex(const std::vector<double> &vals, int &mindex, double &minval) {
	minval = double(INFINITY);
	mindex = -1;

	for (size_t i = 0; i < vals.size(); i++) {
		if (vals[i] < minval) {
			minval = vals[i];
			mindex = int(i);
		}
	}
	if (mindex == -1) {
		return false;
	}
	else {
		return true;
	}
}

bool World::get_index(const std::vector<int> &vals, const int &key, int &index) {
	for (index = 0; index < int(vals.size()); index++) {
		if (vals[index] == key) {
			return true;
		}
	}
	return false;
}



/**************** RETIRED FUNCTIONS *************************************
void World::create_random_obstacles() {
this->obstacle_mat = cv::Mat::zeros(int(this->map_width), int(this->map_height), CV_8UC3);

for (int i = 0; i < this->n_obstacles; i++) {
int r = (rand() %  (this->max_obstacle_radius - this->min_obstacles_radius)) + this->min_obstacles_radius;
cv::Point c;
c.x = rand() % int(this->map_width);
c.y = rand() % int(this->map_height);
cv::circle(this->obstacle_mat, c, r, cv::Vec3b(0,0,100), -1);
}
}

void World::find_obstacle_costs() {
// go through each node and for each of their nbrs, get the obstacle cost for going through each node
for (int i = 0; i < this->n_nodes; i++) {
// get node[i]'s location on the img
cv::Point me = this->nodes[i]->get_loc();
// go through all node[i]'s nbrs
int iter = 0;
int ni = 0;
// get their node[?] index
while(this->nodes[i]->get_nbr_i(iter, ni)) {
// get their location
cv::Point np = this->nodes[ni]->get_loc();
// get a line iterator from me to them
cv::LineIterator lit(this->obstacle_mat, me, np);
double val_sum = 0.0;
// count every obstacles cell between me and them!
for (int i = 0; i < lit.count; i++, ++lit) {
// count along line
if (this->obstacle_mat.at<cv::Vec3b>(lit.pos()) == cv::Vec3b(0,0,100)) {
// hit an obstacle
val_sum++;
}
}
double mean_val = val_sum / double(lit.count);
this->nodes[i]->set_nbr_obstacle_cost(iter, mean_val);
iter++;
}
}

// draw PRM connections
for (int i = 0; i < this->n_nodes; i++) {
int index = -1;
int iter = 0;
while (this->nodes[i]->get_nbr_i(iter, index)) {
iter++; // tracks which nbr I am on
double cost = 0.0;
if (this->nodes[i]->get_nbr_obstacle_cost(iter, cost)) {
cv::line(this->obstacle_mat, this->nodes[i]->get_loc(), this->nodes[index]->get_loc(), cv::Scalar(0,0,0), 5);
cv::Vec3b pink(255.0*(1 - cost), 255.0*(1 - cost), 255);
cv::line(this->obstacle_mat, this->nodes[i]->get_loc(), this->nodes[index]->get_loc(), pink, 2);
}
}
}

// draw nodes
for (int i = 0; i < this->n_nodes; i++) {
cv::circle(this->obstacle_mat, this->nodes[i]->get_loc(), 5, cv::Scalar(255,255,255), -1);
}

// label tasks
for (int i = 0; i < this->n_nodes; i++) {
double d = -5.0;
cv::Point2d l = this->nodes[i]->get_loc();
cv::Point2d tl = cv::Point2d(l.x - d, l.y + d);
char text[10];
sprintf_s(text, "%i", i);
cv::putText(this->obstacle_mat, text, tl, CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,255,255), 3);
}

}
*/
