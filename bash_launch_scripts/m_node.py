#! /usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np
import cv2
import math
import random

from custom_messages.srv import Get_Task_List, Recieve_Agent_Locs, Complete_Work


class Task(object):
	xLoc = -1
	yLoc = -1
	reward = -1
	tp = -1
	work = -1
	work_amounts = [-10, -1]

	def init(self, x,y,r,t,w):
		self.xLoc = x
		self.yLoc = y
		self.reward = r
		self.tp = t
		self.work = w

	def complete_work(self, a_tp):
		try:
			self.work += self.work_amounts[a_tp]
			if self.work <= 0:
				self.work = 0
				self.reward = 0
		except:
			print "master_node.py::task::complete_work::invalid agent type: a_tp=", a_tp

class Agent(object):
	xLoc = -1
	yLoc = -1
	tp = -1
	
	def init(self, x,y,t):
		self.xLoc = x
		self.yLoc = y
		self.tp = t
	
class Master(object):

	generate_replacement_tasks = True
	map_dim = [20, 20]
	n_tasks = 10
	tasks = []
	work_radius = 3
	
	n_agents = 1
	agents = []
	for i in range(0,n_agents):
		a = Agent()
		agents.append(a)

	def init_tasks(self):
		self.tasks = []
		for i in range(0,self.n_tasks):
			self.tasks.append(self.generate_random_task())
	
	def send_task_list(self, trash):
		xLoc = []
		yLoc = []
		reward = []
		for i in range(0,len(self.tasks)):
			xLoc.append(self.tasks[i].xLoc)
			yLoc.append(self.tasks[i].yLoc)
			reward.append(self.tasks[i].reward)

		return [xLoc, yLoc, reward]


	def init_services(self):
		rospy.Service('dmcts_master/get_task_list', Get_Task_List, self.send_task_list)
		rospy.Service('dmcts_master/recieve_agent_locs', Recieve_Agent_Locs, self.recieve_agent_locs)
		rospy.Service('dmcts_master/complete_work', Complete_Work, self.complete_work)

	def print_tasks(self):
		print "*********************Task List********************************"
		for i in range(0,len(self.tasks)):
			print i, ": ", self.tasks[i].xLoc,", ", self.tasks[i].xLoc, " : ", self.tasks[i].reward, " / ", self.tasks[i].work
		print "*********************End Task List********************************"
    	

	def recieve_agent_locs(self, req):
		# who is it, where is the agent and which type are they
		#print req
		self.agents[req.index].init(req.xLoc,req.yLoc,req.type)
		return True
		
	def generate_random_task(self):
		xi = random.randint(0,self.map_dim[0]-1)
		yi = random.randint(0,self.map_dim[1]-1)
		t = Task()
		# xLoc, yLoc, reward_0, type, work_0
		t.init(xi,yi,100,0,100)
		return t

	def complete_work(self, req):#n_index, xLoc, yLoc, a_type):
		#print req
		self.tasks[req.n_index].complete_work(req.a_type)
		# check if the task is complete and remove from list if it is
		if self.tasks[req.n_index].reward <= 0.0:
			print "completed task"
			del self.tasks[req.n_index]
			if self.generate_replacement_tasks:
				self.tasks.append(self.generate_random_task())
			return False

		#m_node.print_tasks()
		if self.dist_2d(req.xLoc, req.yLoc, self.tasks[req.n_index].xLoc, self.tasks[req.n_index].yLoc) < self.work_radius:
			self.tasks[req.n_index].complete_work(req.a_type)
			#m_node.print_tasks()
			return True
		else:
			return False


	def dist_2d(self,ax, ay, bx, by):
		return math.sqrt(pow(ax-bx,2) + pow(ay-by,2))

if __name__ == '__main__':
	rospy.init_node('master')

	m_node = Master()
	m_node.init_tasks()
	m_node.print_tasks()
	m_node.init_services()

#	r = rospy.Rate(1) # 10hz
#	while not rospy.is_shutdown():	
#	    r.sleep()

	rospy.spin()
