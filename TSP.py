#!/usr/bin/python3

from which_pyqt import PYQT_VER
if PYQT_VER == 'PYQT5':
	from PyQt5.QtCore import QLineF, QPointF
elif PYQT_VER == 'PYQT4':
	from PyQt4.QtCore import QLineF, QPointF
else:
	raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))


import time
import numpy as np
from TSPClasses import *
import heapq
import itertools
from queue import PriorityQueue


class TSPSolver:
	def __init__( self, gui_view ):
		self._scenario = None

		self.init_cost = 0
		self.branches = PriorityQueue()
		self.route_matrices = []
		self.routes_in_progress = []
		self.cities_visited = []
		self.costs = []
		self.retBssf = None

		self.count = 0
		self.pruned = 0
		self.starting_cost = 0
		self.states = 0
		self.changed = True
		self.default = True
		self.max_size = 0
    
  def branchAndBound( self, time_allowance=60.0 ):
		randomTour = self.defaultRandomTour()  # O(n) time. This function was pre-defined and not included. Refer to the ReadMe for more info
		self.starting_cost = randomTour.get("cost")		# This function was provided by the source code. It gets a random, valid tour
		self.retBssf = randomTour.get("soln").route
		results = {}	 # dictionary for all the return values
		bssf = None		# bssf will be set as solutions are found
		mat = self.create_matrix()
		reduced_mat, self.init_cost = self.reduce_matrix(mat)	 # get the initial cost and of the tree and the starting reduced matrix
		self.init_priority_queue(reduced_mat)	 # Initialize the priority queue with all the initial branches

		start_time = time.time()		# Start the timer!
		while not self.branches.empty() and time.time() - start_time < time_allowance:		# While there are still branches and the time isn't up
			potential_tuple = self.branches.get()		# Get the potential tuple to branch
			if potential_tuple[0] != float('inf'):		# If the tuple's cost ISNT infinity (meaning it's a valid path)
				self.b_and_b_proper(potential_tuple, start_time, time_allowance)	 # Branch off that branch
			if self.branches.qsize() > self.max_size:
				self.max_size = self.branches.qsize()
		end_time = time.time()		# Get the final time
		if self.retBssf is not None:		# If a valid solution was found during the branch and bound, set this bssf accordingly
			bssf = TSPSolution(self.retBssf)

		# Setting all the return values
		results['cost'] = bssf.cost if self.retBssf is not None else math.inf
		results['time'] = end_time - start_time
		results['count'] = self.count
		results['soln'] = bssf
		results['max'] = self.max_size
		results['total'] = self.states
		results['pruned'] = self.pruned
		return results

	# The FINAL branch and bound algorithm used. Runs in O(n) time (not including method calls).
	# Including method calls, it runs closer to O(n^3)
	def b_and_b_proper(self, route_pair, start_time, time_allowance):
		idx = route_pair[2]
		cities = self._scenario.getCities()		# Gets list of the cities
		mat = self.route_matrices[idx]		# Get the associate matrix for the branch
		route = self.routes_in_progress[idx].copy()		# Get a copy of the route to append to later on
		route_cost = route_pair[1]		# Get the associated cost from the tuple (which determine the priority)
		start_city = route[len(route)-1]	 # Get the starting city from the end of the route
		working_idx = cities.index(start_city)		# Get the index of the city from the list

		# If length is the same then we found valid path!
		if len(route) == len(cities):
			if TSPSolution(route).cost < TSPSolution(self.retBssf).cost:	 # If the bssf exists and the cost is less than the already existing bssf
				self.count += 1
				self.retBssf = route  # Set the route accordingly
			else:
				self.pruned += len(route)
			return
		else:	 # Otherwise we're still searching for a valid path
			# Create all branches and append the new paths and such to the proper channels O(n)
			for i in range(len(mat[working_idx])):	 # For the width of the cost matrix
				if mat[working_idx][i] != float('inf') and cities[i] not in route:	 # If the cost isn't infinity and the city being assessed isn't already in the route
					# Get the associate cost, new reduced matrix, and append the new city onto the route
					new_mat, new_cost = self.reduce_matrix(self.set_inf(working_idx, i, mat))
					route.append(cities[i])
					new_lowerbound = route_cost + mat[working_idx][i] + new_cost
					self.branches.put((len(cities) - len(route), new_lowerbound, len(self.route_matrices)))
					self.route_matrices.append(new_mat)
					self.routes_in_progress.append(route)
					self.cities_visited.append(route)
					self.costs.append(new_lowerbound)
					self.states += 1
					route = self.routes_in_progress[idx].copy()		# Reset the route for the next valid city

	# Initializes the priority queue with the first branches of the tree. O(n) run time
	def init_priority_queue(self, starting_mat):
		cities = self._scenario.getCities()			# Get all the cities
		idx = 0
		for i in range(len(starting_mat[0])):		# For the width of the first row of the matrix (O(n))
			path = [cities[0], cities[i]]		# Make a new path between the stating city and the next
			path_cost = TSPSolution(path).cost		# Get the cost of the path
			if path_cost != float('inf'):		# If the path is valid, the get the associated matrix and cost and append as needed
				inf_mat = self.set_inf(0, i, starting_mat)
				new_mat, new_cost = self.reduce_matrix(inf_mat)
				new_lowerbound = self.init_cost + starting_mat[0][i] + new_cost
        
				self.states += 1
				self.branches.put((len(cities)-len(path), new_lowerbound, idx))
				self.route_matrices.append(new_mat)
				self.routes_in_progress.append(path)
				self.cities_visited.append(path)
				self.costs.append(path_cost)
				idx += 1


	# Sets designed row and column to infinity for further reduction needs
	def set_inf(self, target_row, target_col, mat):
		copy_mat = np.copy(mat)
		for col_val in range(len(copy_mat[target_row])):
			copy_mat[target_row][col_val] = float('inf')

		for row_val in range(len(copy_mat)):
			copy_mat[row_val][target_col] = float('inf')

		copy_mat[target_col][target_row] = float('inf')
		return copy_mat

	# Creates the initial matrix with all the costs between cities
	def create_matrix(self):
		cities = self._scenario.getCities()
		mat = [[0 for x in range(len(cities))] for y in range(len(cities))]
		for city in range(len(cities)):
			for other_city in range(len(cities)):
				route = [cities[city], cities[other_city]]
				mat[city][other_city] = TSPSolution(route).cost
		return mat

	# Reduced the matrix and returns the min_cost and the new matrix
	def reduce_matrix(self, mat):
		min_val = float('inf')
		min_vals = []
		mat_copy = np.copy(mat)
		min_cost = 0

		# Finding and subtracting min values from each row (O(n^2))
		for row in range(len(mat_copy)):
			for col in range(len(mat_copy[row])):
				# If the current value in the ro is smaller than the current minimum
				if mat_copy[row][col] < min_val:
					min_val = mat_copy[row][col]
			# Append the minimum value to an array
			min_vals.append(min_val)
			# If the min cost is not infinite, then add it to the total cost
			if min_val != float('inf'):
				min_cost += min_val
				min_val = float('inf')

		# Subtract the associated minimum values from their designated rows (O(n))
		for row in range(len(mat_copy)):
			for col in range(len(mat_copy[row])):
				# If the matrix value is not infinite
				if mat_copy[row][col] != float('inf'):
					mat_copy[row][col] -= min_vals[row]

		# Finding and subtracting min values from each column
		min_val = float('inf')
		min_vals = []
		for col in range(len(mat_copy)):
			for row in range(len(mat_copy[col])):
				if mat_copy[row][col] < min_val:
					min_val = mat_copy[row][col]
			min_vals.append(min_val)
			if min_val != float('inf'):
				min_cost += min_val
				min_val = float('inf')

		# Subtract the associated minimum values from their designated cols (O(n))
		for col in range(len(mat_copy)):
			for row in range(len(mat_copy[col])):
				if mat_copy[row][col] != float('inf'):
					mat_copy[row][col] -= min_vals[col]

		return mat_copy, min_cost
