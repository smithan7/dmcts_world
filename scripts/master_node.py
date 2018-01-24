#! /usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
import tf
import numpy as np
import cv2
import math
import random

import rviz_tools
from prm import PRM, Agent, Node, Task

from custom_messages.srv import Get_Task_List, Recieve_Agent_Locs, Complete_Work

class Master:
	prm = PRM()
	agents = []
	tasks = []
	missed_reward = 0.0
	collected_reward = 0.0
	markers = rviz_tools.RvizMarkers('/map','visualization_marker')
	br = tf.TransformBroadcaster()

	def init_agents(self, param_file):
		try:
			fs = cv2.FileStorage(param_file, cv2.FILE_STORAGE_READ)
			n_agents= int(fs.getNode('num_agents').real())
			n_types = int(fs.getNode('num_types').real())
			a_works = fs.getNode('agent_work').mat()
			a_starts = fs.getNode('agent_starts').mat()
			a_types = fs.getNode('agent_types').mat()
			for i in range(0,n_agents):
				a = Agent()
				a_tp = a_types[i]
				a.init(i,a_starts[i,0],a_starts[i,1],a_tp, a_works[a_tp])
				self.agents.append(a)
		except:
			print '****************** init_agents::FAILED TO OPEN PARAM FILE *************', param_file

	def init_services(self):
		try:
			rospy.Service('dmcts_master/get_task_list', Get_Task_List, self.send_task_list)
			rospy.Service('dmcts_master/recieve_agent_locs', Recieve_Agent_Locs, self.recieve_agent_locs)
			rospy.Service('dmcts_master/complete_work', Complete_Work, self.complete_work)
		except:
			print '****************** master_node.py::FAILED TO INITIALIZE SERVICES *************'

	def init_timers(self):
		self.rviz_tasks_timer = rospy.Timer(rospy.Duration(1), self.plot_tasks_to_rviz)
		self.rviz_prm_timer = rospy.Timer(rospy.Duration(30), self.plot_prm_to_rviz)
		self.task_timer = rospy.Timer(rospy.Duration(1), self.check_tasks)
		self.broadcast_transform_timer = rospy.Timer(rospy.Duration(1), self.broadcast_transform)

	def broadcast_transform(self):
		self.br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),"/map","world")

	def init_costmap(self, costmap_img_file, param_file):
		try:
			map_img = cv2.imread(costmap_img_file,0)
			fs = cv2.FileStorage(param_file, cv2.FILE_STORAGE_READ)
			self.c2p = fs.getNode('cells_per_pixel').real()
			self.p2c = 1.0/self.c2p
			self.inflation_threshold = fs.getNode('inflation_threshold').real()
			self.costmap = cv2.resize(map_img,None,fx=self.p2c, fy=self.p2c)
			[ret, self.costmap] = cv2.threshold(self.costmap,127,255,cv2.THRESH_BINARY)
			self.map_dim = np.shape(self.costmap)
		except:
			print '****************** init_costmap::FAILED TO OPEN PARAM FILE *************', param_file

	def init_PRM(self, param_file, vertice_file, edge_file):
		self.prm.init(param_file, vertice_file, edge_file)
		print 'PRM Initiated'
		print '   Nodes: ', self.prm.n_nodes
		print '   Edges: ', self.prm.n_edges
		#self.print_tasks

	def cell_to_pixel(self, xc, yc):
		xp = int(xc * self.c2p)
		yp = int(yc * self.c2p)
		return [xp, yp]

	def init_gazebo(self):
		# load SDF model	
		f = open('/home/andy/catkin_ws/src/dmcts_world/worlds/wall_10.sdf','r')
		sdff = f.read()

		cntr = 0
		for i in np.ndenumerate(self.map_img):
			if(i[1]) < 50:
				cntr = cntr + 1
				initial_pose = Pose()
				initial_pose.position.x = i[0][0]
				initial_pose.position.y = i[0][1]
				initial_pose.position.z = 1

				rospy.wait_for_service('gazebo/spawn_sdf_model')
				spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
				name = "robot" + str(cntr)
				spawn_model_prox(name, sdff, "robotos_name_space", initial_pose, "world")

	def inflate_costmap(self):
		work_to_do = True
		done = np.zeros(np.shape(self.costmap),dtype=np.uint8)

		# If I inflated something last time keep going
		while work_to_do:
			work_to_do = False
			working = np.ones(np.shape(self.costmap),dtype=np.uint8)*255
			# go through every pixel
			for xi in range(0,self.map_dim[0]):
				for yi in range(0,self.map_dim[1]):
					# if any adjacent cell val < thresh my val = their val / 2
					if self.costmap[xi][yi] < self.inflation_threshold and done[xi][yi] == 0:
						done[xi][yi] = 1
						nbrs = self.get_nbrs(xi,yi)
						double_val = max(self.costmap[xi][yi]*2,10)
						for ni in nbrs:
							if self.costmap[ni[0]][ni[1]] > double_val and working[ni[0]][ni[1]] > double_val:
								working[ni[0]][ni[1]] = double_val
								work_to_do = True
						nbrs = self.get_diag_nbrs(xi,yi)
						double_val = max(self.costmap[xi][yi]*1.5,10)
						for ni in nbrs:
							if self.costmap[ni[0]][ni[1]] > double_val and working[ni[0]][ni[1]] > double_val:
								working[ni[0]][ni[1]] = double_val
								work_to_do = True
						
			if work_to_do:
				self.costmap = cv2.min(working, self.costmap)


	def get_nbrs(self, xi,yi):
		nbrs = []
		dx = [0,0,-1,1]
		dy = [-1,1,0,0]
		for ii in range(0,4):
			if ii > 1:
				if xi + dx[ii] < self.map_dim[0] and xi+dx[ii] > -1:
					nbrs.append([xi+dx[ii],yi])
			else:
				if yi + dy[ii] < self.map_dim[1] and yi+dy[ii] > -1:
					nbrs.append([xi,yi+dy[ii]])
				
		return nbrs


	def get_diag_nbrs(self, xi,yi):
		nbrs = []
		dx = [-1,-1,1,1]
		dy = [-1,1,-1,1]
		for ii in range(0,4):
			if xi + dx[ii] < self.map_dim[0] and xi+dx[ii] > -1 and yi + dy[ii] < self.map_dim[1] and yi+dy[ii] > -1:
				nbrs.append([xi+dx[ii],yi+dy[ii]])
				
		return nbrs

	def display_costmap_img(self):
		cv2.namedWindow('master::costmap', cv2.WINDOW_NORMAL)
		cv2.imshow('master::costmap',self.costmap)
		cv2.waitKey(0)
		cv2.destroyAllWindows()


	def init_tasks(self, param_file, c_time):

		try:
			fs = cv2.FileStorage(param_file, cv2.FILE_STORAGE_READ)
			p_task_active = fs.getNode('p_task_init_active').real()
			self.work_radius = fs.getNode('work_radius').real()

			for i in range(0,self.prm.n_nodes):
				if random.random() < p_task_active:
					t = Task()
					r0 = 100.0
					t0 = c_time
					t1 = c_time + 200
					t2 = 1
					w0 = 100.0
					t.init(self.prm.nodes[i], r0, t0, t1, t2, w0)
					self.tasks.append(t)

			print len(self.tasks), " Tasks Active"
			#self.print_tasks()
		except:
			print '****************** init_tasks::FAILED TO OPEN PARAM FILE *************', param_file

	def send_task_list(self, trash):

		node_indices = []
		xLoc = []
		yLoc = []
		reward = []
		for t in self.tasks:
			node_indices.append(t.n_index)
			xLoc.append(t.x)
			yLoc.append(t.y)
			reward.append(t.reward)

		return [node_indices, xLoc, yLoc, reward]

	def print_tasks(self):
		print "*********************Task List********************************"
		for t in self.tasks:
			print t.n_index, ": ", t.x,", ", t.y, " : ", t.reward, " / ", t.work
		print "*********************End Task List********************************"
    	
	def recieve_agent_locs(self, req):
		# who is it and where is the agent
		#print req
		self.agents[req.index].update_loc(req.xLoc,req.yLoc)
		return True
		
	def complete_work(self, req):
		try:
			#self.print_tasks()
			if self.dist_2d(req.xLoc, req.yLoc, self.tasks[req.n_index].x, self.tasks[req.n_index].y) < self.work_radius:
				#print "work remaining: ", self.tasks[req.n_index].work
				self.tasks[req.n_index].complete_work(req.a_type, req.c_time)
				#self.print_tasks()
				# check if the task is complete and remove from list if it is
				if self.tasks[req.n_index].work <= 0.0:
					del self.tasks[req.n_index]
					#if self.generate_replacement_tasks:
					#	self.tasks.append(self.generate_random_task())
					return False
				else:
					return True
			else:
				return False
		except:
			print "********************** Unable to do work!!! **************************"
			print req
			self.tasks[req.n_index].print_params()
			#self.print_tasks()
		

	def check_tasks(self, c_time):
		rem_list = []
		for t in self.tasks:
			if not t.task_active(c_time):
				rem_list.append(t)
				self.missed_reward = self.missed_reward + t.clean_up()

		#for r in rem_list:
		#	self.tasks.remove(r)


	def plot_prm_to_rviz(self, event):
		points = []
		for n in self.prm.nodes:
			points.append(Point(n.x,n.y,1))

		d = 0.5
		self.markers.publishSpheres(points, 'white', d, 600.0) # Publish a line between two ROS Point Msgs

		width = 0.1
		for e in self.prm.edges:
			point1 = Point(self.prm.nodes[e.n1].x,self.prm.nodes[e.n1].y,1)
			point2 = Point(self.prm.nodes[e.n2].x,self.prm.nodes[e.n2].y,1)
			self.markers.publishLine(point1, point2, 'white', width, 600.0) # point1, point2, color, width, lifetime

	def plot_tasks_to_rviz(self, event):
		points = []
		for t in self.tasks:
			points.append(Point(t.x,t.y,5))

		d = 2.0
		self.markers.publishSpheres(points, 'red', d, 2.0)

	def dist_2d(self,ax, ay, bx, by):
		return math.sqrt(pow(ax-bx,2) + pow(ay-by,2))


