#!/usr/bin/env python3

from PIL import Image
import numpy as np
from RRT import RRT
import time
from matplotlib import cm
import matplotlib.pyplot as plt

class Score():
	
	def __init__(self):
		# Path to access the map
		# Without obstacles
		self.FILE_PATH = "/home/febin/c_ws/src/autonomous_tb/robot_maps/ground_truth_costmap.pgm"
		# With obstacles
		self.FILE_PATH2 = "/home/febin/c_ws/src/autonomous_tb/robot_maps/obs_costmap.pgm"
		
		# Goal coordinates 
		self.goal  = (203, 275)
		
		# Draw map flag
		self.draw_map = True 
		# Start coordinates
		self.start_coordinates = [(114, 394), (135, 151), (184, 130), (111, 300), (207, 372)]
		self.name_map = {114:'Room 1', 135:'Room 2', 184:'Room 3', 111:'Room 4', 207:'Room 5'}
		# Map scores without obstacles
		self.map_scores_no_obs, self.gradient_map = self.score(self.start_coordinates, self.goal, self.FILE_PATH)
		# Map scores with obstacles
		self.map_scores_obs, self.foo = self.score(self.start_coordinates, self.goal, self.FILE_PATH2)
		
		self.change_dict = dict()
		
		for key in self.map_scores_obs.keys():
			perc_change = ((self.map_scores_obs[key][0] - self.map_scores_no_obs[key][0]) / self.map_scores_no_obs[key][0])*100 
			self.change_dict[self.name_map[key]] = perc_change	
		
		print("Map scores (No obstacles):")		
		print(self.map_scores_no_obs)
		print(" ")
		print("Map scores (With obstacles):")
		print(self.map_scores_obs)
		print(" ")
		print("Percent change in each room:")
		for key in self.change_dict.keys():
			print("{}: {:0.2f}%".format(key, self.change_dict[key]))
		
		# For 3d surface plot
		self.X = np.arange(0, 384)
		self.Y = np.arange(0, 384)
		self.X, self.Y = np.meshgrid(X, Y)
		self.fig, self.ax = plt.subplots(subplot_kw={"projection": "3d"})
		self.surf = self.ax.plot_surface(self.X, self.Y, self.gradient_map, cmap=cm.coolwarm, linewidth=0, antialiased=False)
		#plt.contour(self.gradient_map)
		self.fig.colorbar(surf, shrink=0.5, aspect=5)
		plt.show()	

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
		
	def gkern(self, kernel_size, sigma=1, mu=0):
		x, y = np.meshgrid(np.linspace(-1,1,kernel_size), np.linspace(-1,1,kernel_size))
		d = np.sqrt(x*x+y*y)
		return np.exp(-( (d-mu)**2 / ( 2.0 * sigma**2 ) ) )

	def grad_map(self, map_array, risk_points, kernel_size, sigma, max_cost=10):
		grad_map = map_array
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

	def score(self, locations, goal, file_path):
		"""
		arguments : list containing tuple of start coordinates
				  : goal_coordinates
				  : file_path to the map
		return    : dictionary{start_coordinate[0]: (path_length, path_nodes)} 
		"""
		# Load the map
		map_array = self.load_map(file_path, 1)
		risk_coordinates = [points for points in locations]
		risk_coordinates.append(goal)
		gradient_map = self.grad_map(map_array, risk_coordinates, kernel_size=50, sigma=0.6, max_cost=10)
		gradient_map = np.flipud(gradient_map)
		
		start_time = time.time()
		score_dict = dict()
		for start_coord in locations:
			# Planning class
			RRT_planner = RRT(map_array, start_coord, goal, self.draw_map)
			path_length, path_nodes = RRT_planner.informed_RRT_star(n_pts=2000)
			score_dict[start_coord[0]] = (path_length, path_nodes)
		end_time = time.time()
		print("Total time for calculating scores is %.2f" %(end_time-start_time))
		
		return score_dict, gradient_map	
			

if __name__ == "__main__":
	
	Score()	
	# start = (114, 95) # Room 1, Inf_RRT_star = 143
	# start = (162, 73) # Room 2, Inf_RRT_star = 165
	# start = (91, 246) # Room 4, Inf_RRT_star = 111
	#start = (184, 321) # Room 5, Inf_RRT_star = 101
	# Room 3
	
	
