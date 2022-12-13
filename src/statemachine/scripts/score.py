#!/usr/bin/env python3

from PIL import Image
import numpy as np
from RRT import RRT
import time
from matplotlib import cm
import matplotlib.pyplot as plt
from contextlib import suppress
import scipy

class Score():
	
	def __init__(self):
		# Path to access the map
		# Without obstacles
		self.FILE_PATH = "/home/febin/c_ws/src/autonomous_tb/robot_maps/ground_truth_costmap.pgm"
		# With obstacles
		self.FILE_PATH2 = "/home/febin/c_ws/src/autonomous_tb/robot_maps/obs_costmap.pgm"
		# Change detect map
		self.FILE_PATH_CHANGE = "/home/febin/c_ws/src/autonomous_tb/robot_maps/change_map.png"
		
		# Goal coordinates 
		self.goal  = (203, 275)
		# Start coordinates
		self.start_coordinates = [(114, 394), (135, 151), (184, 130), (111, 300), (207, 372)]
		self.name_map = {114:'Room 1', 135:'Room 2', 184:'Room 3', 111:'Room 4', 207:'Room 5'}
		
		# Draw map flag
		self.draw_map = False 
		
		# Gradient parameters
		self.kernel_size = 50
		self.sigma = 0.6
		self.max_cost = 10
		
		# -------------------Scores using Path length-------------------
		# Scores for map without obstacles
		self.map_scores_no_obs = self.path_score(self.start_coordinates, self.goal, self.FILE_PATH)
		# Scores for map with obstacles
		self.map_scores_obs = self.path_score(self.start_coordinates, self.goal, self.FILE_PATH2)
		# Dictionary to store changes in path length
		self.change_dict = dict()
		# Find perecentage change
		for key in self.map_scores_obs.keys():
			perc_change = ((self.map_scores_obs[key][0] - self.map_scores_no_obs[key][0]) / self.map_scores_no_obs[key][0])*100 
			self.change_dict[self.name_map[key]] = perc_change	
		# Print path length scores 
		print("Map scores (No obstacles):")		
		print(self.map_scores_no_obs)
		print(" ")
		print("Map scores (With obstacles):")
		print(self.map_scores_obs)
		print(" ")
		print("Percent change in each room:")
		for key in self.change_dict.keys():
			print("{}: {:0.2f}%".format(key, self.change_dict[key]))
		
		# -------------------Scores using Gradient map-------------------	
		self.lin_conv_obs = self.grad_score(self.start_coordinates, self.goal, self.FILE_PATH, self.FILE_PATH_CHANGE, self.kernel_size, self.sigma, self.max_cost)
		# Print gardient map score
		print("2D Convolution: ")
		print(self.lin_conv_obs)
			
		

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
		
	def convolve_linear(self, signal, filter, output_size):
		"""
		Linear convolution
		"""
		out = np.zeros(output_size)
		sum = 0

		for i in range(output_size[0]):
			for j in range(output_size[1]):
				for k in range(max(0, i-filter.shape[0]), i+1):
					for l in range(max(0, j-filter.shape[1]), j+1):
						with suppress(IndexError):
							sum += signal[k, l] * filter[i-k, j-l]
				out[i, j] = sum
				sum = 0

		return out	
		
	def gkern(self, kernel_size, sigma=1, mu=0):
		"""
		Gaussian filter
		"""
		x, y = np.meshgrid(np.linspace(-1,1,kernel_size), np.linspace(-1,1,kernel_size))
		d = np.sqrt(x*x+y*y)
		return np.exp(-( (d-mu)**2 / ( 2.0 * sigma**2 ) ) )

	def grad_map(self, grnd_trth_map_path, risk_points, kernel_size, sigma, max_cost=10):
		# Load gradient_map
		grad_map = self.load_map(grnd_trth_map_path, 1)
		kernel = self.gkern(kernel_size=kernel_size, sigma=sigma)*max_cost
		kernel_len = len(kernel)
		steps = int(kernel_len/2.2)
		k_flattened = kernel.flatten()

		for points in risk_points:
		# Center coordinates
			x, y = points[0], points[1]
			i = 0
			for x_coord in range(x-steps, x+steps+1):
				for y_coord in range(y-steps, y+steps+1):
					grad_map[x_coord][y_coord] = grad_map[x_coord][y_coord]*k_flattened[i]
					i+=1
		return grad_map      	

	def path_score(self, locations, goal, file_path):
		# Load the map
		map_array = self.load_map(file_path, 1)
		start_time = time.time()
		score_dict = dict()
		# Run Informed RRT star 
		for start_coord in locations:
			# Planning class
			RRT_planner = RRT(map_array, start_coord, goal, self.draw_map)
			path_length, path_nodes = RRT_planner.informed_RRT_star(n_pts=2000)
			score_dict[start_coord[0]] = (path_length, path_nodes)
		end_time = time.time()
		print("Total time for calculating scores is %.2f" %(end_time-start_time))
		
		return score_dict	
		
	def grad_score(self, locations, goal, grnd_trth_file_path, change_map_file_path, k_size, sig, m_cost):	
		# Load change map
		change_map_array = self.load_map(change_map_file_path, 1)
		
		# List containing all risk points and goal coordinates
		risk_coordinates = [points for points in locations]
		risk_coordinates.append(goal)
		
		# Find gradient map
		gradient_map = self.grad_map(grnd_trth_file_path, risk_coordinates, kernel_size=k_size, sigma=sig, max_cost=m_cost)
		gradient_map = np.flipud(gradient_map)	
		# Crop gradient map
		grad_map_cropped = gradient_map[:-8, :-7]
		# 2D convolution of cropped gradient map and change map
		l_conv = self.convolve_linear(change_map_array, grad_map_cropped, grad_map_cropped.shape)
		
		return(l_conv)
		

if __name__ == "__main__":
	
	Score()	
	
	
