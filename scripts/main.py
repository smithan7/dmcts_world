#! /usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np
import cv2
import math
import random
import tf

from prm import PRM, Agent, Node, Task
from master_node import Master

from custom_messages.srv import Get_Task_List, Recieve_Agent_Locs, Complete_Work

if __name__ == '__main__':
	rospy.init_node('master')
	start_time = rospy.get_time()
	m_node = Master()
	m_node.init_services()
	
	map_name = 'hardware4'
	map_folder = '/home/andy/catkin_ws/src/dmcts_world/worlds/' + map_name + '/'
	

	img_file = map_folder + map_name + '.png'
	param_file = map_folder + map_name + '_params.xml'
	m_node.init_costmap(img_file, param_file)

	## init Gazebo map
	#m_node.init_gazebo()
	
	## initialize agents
	m_node.init_agents(param_file)

	## initialize PRM
	vertice_file = map_folder + map_name + '_vertices.xml'
	edge_file = map_folder + map_name + '_edges.xml'
	m_node.init_PRM(param_file, vertice_file, edge_file)
	m_node.prm.gps_to_pixel(m_node.prm.corners, m_node.map_dim[1], m_node.map_dim[0])
	c_time = rospy.get_time()
	m_node.init_tasks(param_file, c_time)
	m_node.init_services()
	m_node.init_timers()

	loop_rate = rospy.Rate(1) # 1hz
	while rospy.get_time() - start_time < 5:
		print 'waiting'
		loop_rate.sleep()

	#for i in range(0,10):
	#	m_node.plot_prm_to_rviz()

	rospy.spin()