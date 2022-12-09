#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from PIL import Image
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap,GetMapRequest
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from random import randrange
import time

class Explore:

	def __init__(self):
		# Initialize rate
		self.rate = rospy.Rate(1)

		# Simple move_base action client
		self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.move_base.wait_for_server(rospy.Duration(5.0))
		rospy.logdebug("move_base is ready") in the 2 strings with a / or \ depending on the os
		
		# Path dir and file name for saving current map
		self.SAVE_PATH = "/home/febin/RBE-Capstone-main/src/statemachine/current_map.png"
		
		# Map type
		self.map_type = rospy.get_param('map_type', 'house')
		# Dictionary to store map locations
		self.maps = dict()
		# Load goal points
		self.maps = self.map_locations(self.map_type)
		
		# Go to goal points
		self.navigate(self.maps, self.map_type)
		
		# Initialize map_info and map_data variables 
		self.occupancy_grid = OccupancyGrid()
	
		# Initialize subscribers
		self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
		
	def map_callback(self, map_data):
		"""
		Add Map Data to map variable 
		"""
		# Map parameters
		map_width = map_data.info.width
		map_height = map_data.info.height
		map_res = map_data.info.resolution
		
		# Convert occupancy grid data to 8-bit image data
		# -1->255, 0->0, 100->100
		map_data_array = np.flipud(np.resize(np.asarray(map_data.data, dtype='uint8'), (map_height, map_width)))
		
		map_data_array[map_data_array!=100] = 254
		map_data_array[map_data_array==100] = 0 
		
		map_img = Image.fromarray(map_data_array)
		map_img.show()
		map_img.save(self.SAVE_PATH)
		#print(np.unique(map_data_array, return_counts=True))
		
	def loc_pose(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
		"""
		Return Pose(Point(), Quaternion()) of the location
		"""
		self.quat = quaternion_from_euler(roll, pitch, yaw)	 	
		return Pose(Point(x, y, z), Quaternion(self.quat[0], self.quat[1], self.quat[2], self.quat[3]))
		
	def map_locations(self, map_type):
		"""
		Add map goal points to maps dictionary
		Returns dict{'map_type':[All Goal_pose]}
		"""
		self.map_dict = dict()
		self.locations = dict()
		# Define goal points
		self.locations['Point1'] = self.loc_pose(-3.500, -4.000, 0.000, 0, 0, 1.57) 
		
		# Add goal points to maps dictionary
		self.map_dict[map_type] = self.locations
		
		return self.map_dict

	def navigate(self, map_dict, map_type):
		"""
		Navigate through goal points
		"""
		for location in self.map_dict[self.map_type].keys():
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose = self.map_dict[self.map_type][location]
			print("Moving to goal:")
			print(goal.target_pose.pose)
			self.move_base.send_goal(goal)
			wait = self.move_base.wait_for_result()
			if not wait:
				rospy.logerr("Action server down")
			else:
				print("Reached Goal!")
				print(" ")	 
		
		
def main():
	""" The main() function """
	rospy.init_node('explore')
	#rospy.init_node('explore', log_level=rospy.DEBUG)
	Explore()

if __name__ == '__main__':
	try:
		main()
	except:
		rospy.ROSInterruptException
