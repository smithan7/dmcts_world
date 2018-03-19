#!/usr/bin/env python

import csv
import random
import os

if __name__ == '__main__':

    num_nodes = 100
    coord_type_set = ['"mcts_task_by_completion_reward_gradient"']#, '"greedy_completion_reward"']

    num_agents = 5
    coord_type = "mcts_task_by_completion_reward_gradient"
    p_active = 0.5

    use_gazebo = False
    maps_per_round = 3

    depth_set = [8]#[4,5,6,7,8]
    beta_set = [1.5]#[0.5, 1.0, 1.5]
    alpha_set = [0.1, 0.25, 0.5, 0.75, 0.9, 0.95, 0.99, 1.0]
    param_set = [34, 50, 81]

    for depth in depth_set:
        for beta in beta_set:
            for alpha in alpha_set:
                
                os.system('rm /home/andy/catkin_ws/src/dmcts_world/worlds/results.csv')
                os.system('touch /home/andy/catkin_ws/src/dmcts_world/worlds/results.csv')


                for param in param_set:
                    if use_gazebo:
                        command = '/home/andy/catkin_ws/src/dmcts_world/bash_launch_scripts/n_controlled_quads.sh ' + str(num_agents) + ' ' + str(num_nodes) + ' ' + str(param) + ' ' + coord_type + ' ' + str(p_active)
                    else:
                        command = '/home/andy/catkin_ws/src/dmcts_world/bash_launch_scripts/n_controlled_simulated_quads_params.sh ' + str(num_agents) + ' ' + str(num_nodes) + ' ' + str(param) + ' ' + coord_type + ' ' + str(p_active) + ' ' + str(alpha) + ' ' + str(beta) + ' ' + str(depth) 
                        #command = '/home/andy/catkin_ws/src/dmcts_world/bash_launch_scripts/n_controlled_fake_quads_params.sh 2 100 ' + str(iter) + ' "greedy_completion_reward" 0.25'
        			
                    print("sending bash command: " + command)
            
                    os.system(command)
                    os.system('sleep 5s')
                    os.system('rosnode kill -a')
                    os.system('sleep 5s')
    
                results = []
                n_rows = 0.0
                test_score = 0.0
                with open('/home/andy/catkin_ws/src/dmcts_world/worlds/results.csv', 'rb') as resultsfile:
                    r = csv.reader(resultsfile)
                    for i, row in enumerate(r,1):
                        test_score += float(row[0])
                        results.append(float(row[0]))
                        n_rows += 1.0
                        
                print("Results found: " + str(n_rows))
                test_score /= n_rows
        
                os.system('rm /home/andy/catkin_ws/src/dmcts_world/worlds/results.csv')
        
                print("test_values mean score: " + str(test_score))
                
                with open('/home/andy/catkin_ws/src/dmcts_world/worlds/5_agent_trainer_results.csv', 'a') as csvfile:
                    w= csv.writer(csvfile, delimiter=',')
                    tt = [str(test_score), str(alpha), str(beta), str(depth), results]
                    w.writerow(tt)