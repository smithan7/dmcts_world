#include "Agent_Coordinator.h"

#include "Proabability_Node.h"
#include "World.h"
#include "Agent.h"
#include "Goal.h"
#include "Map_Node.h"

#include <iostream>
#include <ros/ros.h>

Agent_Coordinator::Agent_Coordinator(Agent* agent, int n_tasks) {
	this->agent = agent;
	this->n_tasks = n_tasks;
	for (int i = 0; i < this->n_tasks; i++) {
		Probability_Node* pn = new Probability_Node(i);
		this->prob_actions.push_back(pn);
	}

	this->task_claim_method = agent->get_task_claim_method();
	this->task_claim_time = agent->get_task_claim_time();
}


Agent_Coordinator::~Agent_Coordinator() {
	for (int i = 0; i < this->n_tasks; i++) {
		delete this->prob_actions[i];
	}
}

void Agent_Coordinator::reset_prob_actions() {
	for (int i = 0; i < this->n_tasks; i++) {
		if (this->prob_actions[i]->claimed()) {
			this->prob_actions[i]->reset();
		}
	}
}

bool Agent_Coordinator::get_advertised_task_claim_probability(const int &task_num, const double &query_time, double &prob_taken, World* world) {
	std::cerr << "Agent_Coordinator::get_advertised_task_claim_probability: in" << std::endl;
	if (task_num < 0 || task_num > this->n_tasks) {
		return false;
	}

	if (this->task_claim_time.compare("arrival_time") == 0) {
		// claim task at the time of arrival
		prob_taken = world->get_team_probability_at_time_except(query_time, task_num, this->agent->get_index());
		return true;
	}
	else if (this->task_claim_time.compare("completion_time") == 0) {
		// claim task at the time task is completed
		prob_taken = world->get_team_probability_at_time_except(query_time, task_num, this->agent->get_index());
		return true;
	}
	else if (this->task_claim_time.compare("immediate") == 0) {
		// checks if there are any orders on the book right now, doesn't care about the future
		prob_taken = world->get_team_probability_at_time_except(query_time, task_num, this->agent->get_index());
		return true;
	}
	else if (this->task_claim_time.compare("none") == 0) {
		prob_taken = 0.0;
		return true;
	}
	else {
		std::cerr << "Agent_Coordinator::coordinate_task_selection: No method specified" << std::endl;
		return false;
	}
}

bool Agent_Coordinator::get_claims_after(const int &task, const double &query_time, std::vector<double> &probs, std::vector<double> &times) {
	// if there are claims after the query time, add to list and return true, exclude inf
	if (this->prob_actions[task]->get_claims_after(query_time, probs, times)){
		return true;
	}
	else {
		return false;
	}
}

double Agent_Coordinator::get_reward_impact(const int &task_i, const int &agent_i, const double &completion_time, World* world) {
	// idea is to identify the expected impact of the agent taking this task
	// i.e. how much difference will it make, impact = R(completion_time) - Expectation[ R(completion_time -> inf)]

	// need to know the reward function
	// need to know the expected time of arrival of all other agents arriving after me

	Map_Node* task = world->get_nodes()[task_i];
	Agent* agent = world->get_agents()[agent_i];

	double my_reward = task->get_reward_at_time(completion_time);

	if (my_reward <= 0.0) {
		return 0.0;
	}
	
	if (world->get_impact_style() == "before_and_after" || world->get_impact_style() == "optimal" || world->get_impact_style() == "fixed") {
		double p_taken = 0.0;
		if (this->agent->get_coordinator()->get_advertised_task_claim_probability(task_i, completion_time, p_taken, world)) {
			my_reward *= (1 - p_taken);
		}
	}
		

	// go through each agent's coordinator and check this task for claims AFTER my completion time.
	Probability_Node shared_plan(task_i);
	for (int a = 0; a < world->get_n_agents(); a++) {
		if (agent_i != a) {
			Agent_Coordinator* coord = world->get_agents()[a]->get_coordinator();
			// get all active claims by each agent and add to the shared plan
			std::vector<double> p, t;
			if (coord->get_claims_after(task_i, completion_time, p, t)) {
				for (size_t i = 0; i < p.size(); i++) {
					shared_plan.add_stop_to_shared_path(t[i], p[i]);
				}
			}
		}
	}

	double c_reward = 0;
	// if they have a claim, get the P(complete | time) * reward(time)
	std::vector<double> t, p;
	if (shared_plan.get_claims_after(completion_time, p, t)) {
		// there are claims after mine, remvoe reward they will (probably) get

		for (size_t c = 0; c < p.size(); c++) {
			// here p[c] = p[t_c] - p[t_{c-1}]
			c_reward += p[c] * task->get_reward_at_time(t[c]);
		}
	}

	// impact is my reward - the reward other agents will get if I don't get it right now
	double impact = std::max(my_reward - c_reward, 0.0);

	return impact;
}


void Agent_Coordinator::print_prob_actions() {
	int agent_index = this->agent->get_index();

	printf("Agent %i ************************************************ \n", agent_index);
	for (size_t i = 0; i < this->prob_actions.size(); i++) {
		this->prob_actions[i]->print_out();
	}

}

void Agent_Coordinator::add_stop_to_my_path(const int &task_index, const double &time, const double &prob){
	if (task_index > this->prob_actions.size()) {
		return;
	}
	
	this->prob_actions[task_index]->add_stop_to_my_path(time, prob);
};

bool Agent_Coordinator::advertise_task_claim(World* world) {

	if (this->agent->get_goal()->get_index() < 0 || this->agent->get_goal()->get_index() > this->n_tasks) {
		return false;
	}

	double prob_taken = 1.0;
	if (this->task_claim_method.compare("greedy") == 0) {
		prob_taken = 0.99;
	}
	else {
		std::cerr << "Agent_Coordinator::advertise_task_claim: no method specified" << std::endl;
	}

	if (this->task_claim_time.compare("arrival_time") == 0) {
		// claim task at the time of arrival
		this->prob_actions[this->agent->get_goal()->get_index()]->add_stop_to_my_path(this->agent->get_goal()->get_arrival_time(), prob_taken);
		return true;
	}
	else if (this->task_claim_time.compare("completion_time") == 0) {
		// claim task at the time it will be completed
		this->prob_actions[this->agent->get_goal()->get_index()]->add_stop_to_my_path(this->agent->get_goal()->get_completion_time(), prob_taken);
		return true;
	}
	else if (this->task_claim_time.compare("first_bid") == 0) {
		// checks if there are any orders on the book right now, doesn't care about the future
		this->prob_actions[this->agent->get_goal()->get_index()]->add_stop_to_my_path(world->get_c_time(), prob_taken);
		return true;
	}
	else if (this->task_claim_time.compare("none") == 0) {
		prob_taken = 0.0;
		return true;
	}
	else {
		std::cerr << "Agent_Coordinator::coordinate_task_selection: No method specified" << std::endl;
		return false;
	}
}