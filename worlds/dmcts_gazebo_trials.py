#!/usr/bin/env python

import csv
import random
import os

if __name__ == '__main__':

    num_agent_set = [3]#[2,3,5,7]
    p_active_set = [0.25]#[0.25, 0.25, 0.4, 0.5, 0.7]
    num_nodes = 100
    coord_type_set = ['"mcts_task_by_completion_reward_gradient"', '"greedy_completion_reward"']

    use_gazebo = True
    maps_per_round = 1

    for ai in range(0,len(num_agent_set)):
        for ct in range(0,2):
    
            num_agents = num_agent_set[ai]
            coord_type = coord_type_set[ct]#"mcts_task_by_completion_reward_gradient" "greedy_completion_reward"
            p_active = p_active_set[ai]
            
            os.system('rm /home/andy/catkin_ws/src/dmcts_world/worlds/temp_results.csv')
            os.system('touch /home/andy/catkin_ws/src/dmcts_world/worlds/temp_results.csv')

            for iter in range(1,maps_per_round+1):
                best_row = []
                with open('/home/andy/catkin_ws/src/dmcts_world/worlds/params.csv', 'r') as csvfile:
                    r = csv.reader(csvfile)
                    best_score = 0.0
                    for row in r:
                        if float(row[6]) > best_score:
                            best_score = float(row[6])
                            best_row = row
    
                print ("best params: " + str(best_row[0:-1]) + " with score " + str(best_score) + " maps per round: " + str(maps_per_round))
                
                temp_param = []
                for param in best_row:
                    if(random.random() > 1.25):
                        temp_param.append(float(param) + float(param) * 0.25*(random.random()-0.5))
                    else:
                        temp_param.append(float(param))
                
                rando = random.random()
                if rando > 0.75:
                    temp_param[0] = float(best_row[0]) + 1
                elif rando > 0.5:
                    temp_param[0] = float(best_row[0]) - 1
                else:
                    temp_param[0] = float(best_row[0])
    
                # Restrict Gamma
                temp_param[3] = min(temp_param[3], 0.9999999)
    
                print("test_values: " + str(temp_param[0:-1]))
                
                if use_gazebo:
                    command = '/home/andy/catkin_ws/src/dmcts_world/bash_launch_scripts/n_controlled_quads.sh ' + str(num_agents) + ' ' + str(num_nodes) + ' ' + str(iter) + ' ' + coord_type + ' ' + str(p_active)
                else:
                    command = '/home/andy/catkin_ws/src/dmcts_world/bash_launch_scripts/n_controlled_fake_quads_params.sh ' + str(num_agents) + ' ' + str(num_nodes) + ' ' + str(iter) + ' ' + coord_type + ' ' + str(p_active)
                    #command = '/home/andy/catkin_ws/src/dmcts_world/bash_launch_scripts/n_controlled_fake_quads_params.sh 2 100 ' + str(iter) + ' "greedy_completion_reward" 0.25'
                
                for t in temp_param: ## add the params to the command
                    command += ' ' + str(t)
    			
                print("sending bash command: " + command)
    
                os.system(command)
                os.system('sleep 5s')
                os.system('rosnode kill -a')
                os.system('sleep 15s')
    
            results = []
            n_rows = 0.0
            test_score = 0.0
            with open('/home/andy/catkin_ws/src/dmcts_world/worlds/temp_results.csv', 'rb') as resultsfile:
                r = csv.reader(resultsfile)
                for i, row in enumerate(r,1):
                    test_score += float(row[0])
                    n_rows += 1.0
                    temp_param.append(float(row[0]))
                    results.append(float(row[0]))
                    
            print("Results found: " + str(n_rows))
            test_score /= n_rows
    
            os.system('rm /home/andy/catkin_ws/src/dmcts_world/worlds/temp_results.csv')
    
            print("test_values mean score: " + str(test_score))
            temp_param[6] = test_score
            temp_param[7] = maps_per_round
    
            temp_param.append('This is the first test with varied reward types for the agents, 60s')
    
            with open('/home/andy/catkin_ws/src/dmcts_world/worlds/gazebo_results.csv', 'a') as csvfile:
                w= csv.writer(csvfile, delimiter=',')
                tt = [str(test_score), str(iter), str(num_agents), str(num_nodes), str(p_active), coord_type, results]
                w.writerow(tt)