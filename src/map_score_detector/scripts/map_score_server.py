

from __future__ import print_function
import rospy
import sys
sys.path.append('/home/deveshdatwani/capstone/src/map_score_detector/scripts/')
from detect_changes import ChangeDetector


class MapChangeServer():

	def __init__(self):
		self.server_name = "map_changer_server"
		self.service_type = ChangeDetector

	def find_changes(self, image1, image2):
		detector = ChangeDetector()
		detected_change = detector.find_PCAKmeans(image1, image2)
		return detected_change


if __name__ == "__main__":
	pass