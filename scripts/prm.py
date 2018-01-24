#! /usr/bin/env python
import rospy
import numpy as np
import cv2
import math
import random

import utils

class Task:
	n_index = -1
	x = -1
	y = -1
	reward = -1
	
	t = -1
	work = -1
	start_time = -1
	end_time = -1
	work_amounts = [-10, -1]


	def init(self,node, r0, t0, t1, t2, w0):
		self.x = node.x
		self.y = node.y
		self.t = node.t
		self.n_index = node.i
		self.work = w0
		self.reward = r0
		self.start_time = t0
		self.end_time = t1
		self.window_type = t2
		self.work_amounts = [-10, -10]

	def complete_work(self, a_tp, c_time):
		try:
			self.work += self.work_amounts[a_tp]
			if self.work <= 0.0:
				self.work = 0.0
		except:
			print "master_node.py::task::complete_work::invalid agent type: a_tp=", a_tp

	def calc_reward(self, c_time):
		return self.reward

	def task_active(self, time):
		#print self.n_index, ": ", self.start_time, ' : ', time, ' : ', self.end_time
		if time > self.start_time and time < self.end_time and self.work > 0:
			return True
		else:
			return False

	def clean_up(self):
		if self.work > 0.0:
			return self.reward
		else:
			return 0.0

	def print_params(self):
		print 'Task Node: ', self.n_index
		print '   Loc: ',self.x, ', ', self.y
		print '   Work: ', self.work
		print '   Reward: ', self.reward
		print '   Start Time: ', self.start_time


class Agent:
	i = -1
	x = -1
	y = -1
	tp = -1

	def init(self, i, x,y,t,w):
		self.i = i
		self.x = x
		self.y = y
		self.tp = t
		self.w = w

	def update_loc(self, x, y):
		self.x = x
		self.y = y

	def print_stats(self):
		print 'agent[', self.i,']'
		print '   loc: (',self.x,', ',self.y,')'
		print '   type: ',self.tp
		print '   work: ',self.w

class Node:
	i = -1
	x = -1
	y = -1
	t = -1
	nbrs = []
	dists = []

	def init(self,i,x,y,t,nbrs,dists):
		self.i = i
		self.x = x
		self.y = y
		self.t = t
		self.nbrs = nbrs
		self.dists = dists

	def print_params(self):
		print 'Node[',self.i,']'
		print '   loc: ',self.x,', ',self.y
		print '   type: ',self.t
		print '   nbrs: ', self.nbrs
		print '   dists: ', self.dists

class Edge:
	n1 = -1
	n2 = -1
	free_cost = -1
	obs_cost = -1

	def init(self, n1, n2, f, o):
		self.n1 = n1.i
		self.n2 = n2.i
		self.free_cost = f
		self.obs_cost = o

class PRM:
	nodes = []
	edges = []

	def init(self, param_file, vertice_file, edge_file):
		try:
			n_nbrs = []
			n_dists = []
			types = [0]
			vf = cv2.FileStorage(vertice_file, cv2.FILE_STORAGE_READ)
			#help(cv2.FileNode())
			self.n_nodes = int(vf.getNode('n_vertices').real())
			n_corners = vf.getNode('corners').size()
			self.corners = []
			for i in range(0,n_corners):
				self.corners.append(vf.getNode('corners').at(i).real())
			
			for i in range(0,self.n_nodes):
				n_str = 'vertex'+ str(i) +'_gps'
				nx = vf.getNode(n_str).at(0).real()
				ny = vf.getNode(n_str).at(1).real()
				n = Node()
				n.init(i,nx,ny,types[0],n_nbrs,n_dists)
				self.nodes.append(n)

			ef = cv2.FileStorage(edge_file, cv2.FILE_STORAGE_READ)
			
			self.n_edges = int(ef.getNode('n_edges').real())
			for i in range(0,self.n_edges):
				n_str = 'edge'+ str(i)
				n0 = int(ef.getNode(n_str).at(0).real())
				n1 = int(ef.getNode(n_str).at(1).real())
				o_cost = ef.getNode(n_str).at(2).real()
				f_cost = ef.getNode(n_str).at(4).real()
				e = Edge()
				e.init(self.nodes[n0],self.nodes[n1], f_cost, o_cost)
				self.edges.append(e)
		except:
			print '****************** init_PRMS::FAILED TO OPEN PARAM FILE *************', vertice_file

	def print_params(self):
		for n in self.nodes:
			n.print_params()

	def gps_to_pixel(self, corners, img_width, img_height):
		
		if corners[0] < corners[2]:
			west = corners[0]
			east = corners[2]
			gps_width = abs(west-east)
		else:
			west = corners[2]
			east = corners[0]
			gps_width = abs(west-east)
		
		if corners[1] < corners[3]:
			north = corners[3]
			south = corners[1]
			gps_height = abs(north-south)
		else:
			north = corners[1]
			south = corners[3]
			gps_height = abs(north-south)

		for n in self.nodes:
			n.x = img_width * (west - n.x) / gps_width
			n.x = n.x / 10.0
			n.y = img_height * (north - n.y) / gps_height
			n.y = n.y / 10.0

		for e in self.edges:
			r = e.obs_cost / e.free_cost
			e.free_cost = math.sqrt(pow(self.nodes[e.n1].x-self.nodes[e.n2].x,2) + pow(self.nodes[e.n1].y-self.nodes[e.n2].y,2) )
			e.obs_cost = r * e.free_cost












		


