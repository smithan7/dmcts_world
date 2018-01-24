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

	def init(self, x,y,r):
		self.xLoc = x
		self.yLoc = y
		self.reward = r

class Slave(object):
	tasks = []


	def get_task_list_client(self):
	    rospy.wait_for_service('get_task_list')
	    try:
	        get_task_list_fnctn = rospy.ServiceProxy('get_task_list', Get_Task_List)
	        resp1 = get_task_list_fnctn()
	        #print resp1
	    except rospy.ServiceException, e:
	        print "Service call failed: %s"%e
	
	def report_location(self):
	    rospy.wait_for_service('recieve_agent_locs')
	    try:
	        report_location_fnctn = rospy.ServiceProxy('recieve_agent_locs', Recieve_Agent_Locs)
	        # index, xLoc, yLoc, type
	        resp1 = report_location_fnctn(0,10,9,0)
	        #print resp1
	    except rospy.ServiceException, e:
	        print "Service call failed: %s"%e

	def do_work(self):
	    rospy.wait_for_service('complete_work')
	    try:
	        complete_work_fnctn = rospy.ServiceProxy('complete_work', Complete_Work)
	        # index, xLoc, yLoc, type
	        c_time = rospy.get_time()
	        resp1 = complete_work_fnctn(0, 10, 9, 0, c_time)
	        #print resp1
	    except rospy.ServiceException, e:
	        print "Service call failed: %s"%e

	def print_tasks(self):
		for i in range(0,len(self.tasks)):
			print self.tasks[i].loc, ": ", self.tasks[i].reward
    	

if __name__ == '__main__':
	rospy.init_node('slave')

	s_node = Slave()
	s_node.get_task_list_client()

	r = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		#s_node.report_location()
		s_node.do_work()
		r.sleep()

	rospy.spin()
