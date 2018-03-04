#!/usr/bin/env python

import cv2

import rospy, math
import numpy as np
import random
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from decimal import Decimal



if __name__ == '__main__':
	seed = 1
	n_agents = 4
	coord = 'greedy_completion_reward'

	tree = ET.parse('/home/andy/catkin_ws/src/dmcts_world/worlds/results/results_for_param_file_1_4_greedy_completion_reward.xml')
	root = tree.getroot()

	num_nodes = 0.0
	num_agents = 0.0
	rewards = []
	times = []
	cumulative_reward = 0.0
	param = 0.0

	for i, child in enumerate(root,1):
		if child.tag == 'num_nodes':
			num_nodes = float(root[i-1].text)
		elif child.tag == 'num_agents':
			num_agents = float(root[i-1].text)
		elif child.tag == 'reward':
			rewards = root[i-1].text
		elif child.tag == 'time':
			times = root[i-1].text
		elif child.tag == 'cumulative_reward':
			cumulative_reward = float(root[i-1].text)

	print "num_agents: ", num_agents
	print "num_nodes: ", num_nodes
	print "param: ", param
	print "cumulative_reward: ", cumulative_reward
	print "reward: ", rewards
	print "times: ", times

	print len(rewards)
	print rewards[0], '*****************'
	times = np.asarray(times)
	rewards = np.asarray(rewards)

	fig, ax = plt.subplots()
	ax.plot(times, rewards)

	ax.set(xlabel='time (s)', ylabel='voltage (mV)',
	       title='About as simple as it gets, folks')
	ax.grid()

	fig.savefig("test.png")
	plt.show()




