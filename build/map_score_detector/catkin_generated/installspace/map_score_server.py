#!/usr/bin/env python3

from __future__ import print_function
import rospy
from map_score_detector.srv import *
from change_detector import ChangeDetector


class MapChangeServer():

	def __init__(self):
		self.server_name = "map_changer_server"
		self.service_type = ChangeDetector

	def find_changes(self, req):
		detector = ChangeDetector()
		detected_change = detector.find_PCAKmeans(req.image1, req.image2)
		return detected_changes


	def start_server(self):
		rospy.init_node('change_map_server')
		service = rospy.Service("detect_map_change", ChangeDetector, self.find_changes)
		print("Map CHange Server Is Running!!")
		rospy.spin()


if __name__ == "__main__":
	s = MapChangeServer()
	s.start_server()






