#!/usr/bin/env python3

from PIL import Image
import numpy as np
from RRT import RRT
import time

import matplotlib.pyplot as plt

class Score():
	
	def __init__(self):
		# Path to access the map
		self.FILE_PATH = "/home/febin/RBE-Capstone-main/src/statemachine/maps/map.pgm"
		
		# Goal coordinates 
		self.goal  = (185, 220)
		
		# Start coordinates
		self.start_coordinates = [(114, 95), (162, 73), (91, 246), (184, 321)]
		
		self.map_scores = self.score(self.start_coordinates, self.goal, self.FILE_PATH)
		print(self.map_scores)		

	def load_map(self, file_path, resolution_scale):
		''' Load map from an image and return a 2D binary numpy array
			where 0 represents obstacles and 1 represents free space
		'''
		# Load the image with grayscale
		img = Image.open(file_path).convert('L')
		# Rescale the image
		size_x, size_y = img.size
		new_x, new_y  = int(size_x*resolution_scale), int(size_y*resolution_scale)
		img = img.resize((new_x, new_y), Image.ANTIALIAS)

		map_array = np.asarray(img, dtype='uint8')

		# Get binary image
		threshold = 127
		map_array = 1 * (map_array > threshold)	
		# Result 2D numpy array
		return map_array

	def score(self, locations, goal, file_path):
		"""
		arguments : list containing tuple of start coordinates
				  : goal_coordinates
				  : file_path to the map
		return    : dictionary{start_coordinate[0]: (path_length, path_nodes)} 
		"""
		# Load the map
		map_array = self.load_map(file_path, 1)
		start_time = time.time()
		score_dict = dict()
		for start_coord in locations:
			# Planning class
			RRT_planner = RRT(map_array, start_coord, goal)
			path_length, path_nodes = RRT_planner.informed_RRT_star(n_pts=1500)
			score_dict[start_coord[0]] = (path_length, path_nodes)
		end_time = time.time()
		print("Total time for calculating scores is %.2f" %(end_time-start_time))
		return score_dict	
			

if __name__ == "__main__":
	
	Score()	
	# start = (114, 95) # Room 1, Inf_RRT_star = 143
	# start = (162, 73) # Room 2, Inf_RRT_star = 165
	# start = (91, 246) # Room 4, Inf_RRT_star = 111
	#start = (184, 321) # Room 5, Inf_RRT_star = 101
	# Room 3
	
	
