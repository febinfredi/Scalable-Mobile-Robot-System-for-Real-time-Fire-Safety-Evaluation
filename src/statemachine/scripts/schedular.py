import time
import rospy 
import sys
from matplotlib import pyplot as plt
sys.path.append('/home/deveshdatwani/capstone/src/map_score_detector/scripts/')
from map_score_server import MapChangeServer 

class Scheduler():

    def __init__(self):
        self.state = 0
        self.ground_truth_map = "/home/deveshdatwani/capstone/src/map_score_detector/examples/image1.jpeg"
        self.current_map = None
        self.time = time.time() 
        self.state_codes = {-1:"shutdown", 
                            0:"rest",
                            1: "ready",
                            2: "battery dead",
                            3: "",
                            4: "",
                            5: "compare cost maps"}
        self.map_change_detector_routine =  MapChangeServer()

    def run(self):
        
        while self.state > -1:

            if self.state == 0:
                time.sleep(1)
                print(f'Current state is {self.state_codes[self.state]}') 
                if time.time() - self.time > 1:
                    self.state = 1
                    self.time = time.time()

            elif self.state == 1:
                time.sleep(1)
                print("Bot is ready to run")
                if time.time() - self.time > 1:
                    self.state = 5 
                # WAYPOINT NAVIGATION HERE
                # SAVE NEW MAP TO EXAMPLES 

            elif self.state == 5:
                print("Calculate change map")
                #create a service call to the change detector
                self.current_map = "/home/deveshdatwani/capstone/src/map_score_detector/examples/image2.jpeg"
                changed_map = self.map_change_detector_routine.find_changes(self.ground_truth_map, self.current_map)
                self.state = -1
                plt.imshow(changed_map)
                plt.show()
                
        print("Shutdown")


if __name__ == "__main__":
    schedular = Scheduler()
    schedular.run()

