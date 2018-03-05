#!/usr/bin/env python

import csv
import random
import os

if __name__ == '__main__':


    number_of_rounds = 100
    for test_round in range(1,number_of_rounds+1):

        os.system('rm /home/andy/catkin_ws/src/dmcts_world/worlds/results.csv')
        os.system('touch /home/andy/catkin_ws/src/dmcts_world/worlds/results.csv')

        maps_per_round = 5
        for iter in range(1,maps_per_round+1):
            best_row = []
            with open('/home/andy/catkin_ws/src/dmcts_world/worlds/params.csv', 'r') as csvfile:
                r = csv.reader(csvfile)
                best_score = 0.0
                for row in r:
                    if float(row[6]) > best_score and float(row[7]) == float(maps_per_round):
                        best_score = float(row[6])
                        best_row = row

            print ("best params: " + str(best_row[0:-1]) + " with score " + str(best_score) + " maps per round: " + str(maps_per_round))
            
            temp_param = []
            for param in best_row:
                if(random.random() > 0.25):
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
            command = '/home/andy/catkin_ws/src/dmcts_world/bash_launch_scripts/n_controlled_fake_quads_params.sh 2 100 ' + str(iter) + ' "mcts_task_by_completion_reward_gradient" 0.25'
            for t in temp_param:
                command += ' ' + str(t)
			
            print("sending bash command: " + command)

            os.system(command)
            os.system('sleep 5s')
            os.system('rosnode kill -a')
            os.system('sleep 5s')

        n_rows = 0.0
        test_score = 0.0
        with open('/home/andy/catkin_ws/src/dmcts_world/worlds/results.csv', 'rb') as resultsfile:
            r = csv.reader(resultsfile)
            for i, row in enumerate(r,1):
                test_score += float(row[0])
                n_rows += 1.0

        print("Results found: " + str(n_rows))
        test_score /= n_rows

        os.system('rm /home/andy/catkin_ws/src/dmcts_world/worlds/results.csv')

        print("test_values mean score: " + str(test_score))
        temp_param[6] = test_score
        temp_param[7] = maps_per_round

        with open('/home/andy/catkin_ws/src/dmcts_world/worlds/params.csv', 'a') as csvfile:
            w= csv.writer(csvfile, delimiter=',')
            w.writerow(temp_param)