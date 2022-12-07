import time
import rospy 
import sys
from matplotlib import pyplot as plt
from map_score_server import MapChangeServer
#from goal_sequence import Explore

class Scheduler():

    def __init__(self):
        self.state = 0
        self.ground_truth_map = "/home/deveshdatwani/capstone/src/statemachine/robot_map/ground_truth_map.jpeg"
        self.current_map = "/home/deveshdatwani/capstone/src/statemachine/robot_map/current_map.jpeg"
        self.time = time.time()
        self.ideal_score = None 
        self.state_codes = {-1:"shutdown", 
                            0:"rest",
                            1: "explore",
                            2: "scoring",
                            3: "",
                            4: "",
                            5: "compare cost maps"}
        #self.map_change_detector_routine =  Explore()


    def run(self):
        
        while self.state > -1:


            # 1. ROBOT RESTING
            if self.state == 0:
                time.sleep(1)
                print(f'Current state is {self.state_codes[self.state]}') 
                if time.time() - self.time > 0.5:
                    self.state = 1
                    self.time = time.time()


            # 2. ROBOT EXPLORE        
            elif self.state == 1:
                time.sleep(1)
                print("Bot is ready to run")
                if time.time() - self.time > 1:
                    self.state = 2 
                # WAYPOINT NAVIGATION HERE
                # SAVE NEW MAP TO /ROBOT_MAPS 


            # 3. ROBOT SAFETY SCORE THROUGH INFORMED RRT STAR
            elif self.state == 2:
                print(f'calculating score with for {self.state_codes[self.state]}')
                self.state = 3 


            # 4. ROBOT CHANGE DETECTION WITH PCA & KMEANS          
            elif self.state == 3:
                print(f'Calculate change map at state {self.state_codesp[self.state]}')
                #create a service call to the change detector
                changed_map = self.map_change_detector_routine.find_changes(self.ground_truth_map, self.current_map)
                self.state = 4
                plt.imshow(changed_map)
                plt.show()


            # 5. NOTIFY USER WITH MOBILE APPLICATION
            elif self.state == 4:
                self.state = None                
        
        
        # 6. IF NOT STATE IS ACTIVE / BATTERY DOWN
        print("Shutdown")


if __name__ == "__main__":
    schedular = Scheduler()
    schedular.run()

