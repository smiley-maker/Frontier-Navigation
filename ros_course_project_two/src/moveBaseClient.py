#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

#Action Client class
class MoveBaseClient:
    #Constructor to setup the initialization and callback functions. 
    def __init__(self):
        self.init_node()
        self.init_action_client()
    
    def init_node(self):
        rospy.init_node("moving") #Creates a new node called \moving. 
    
    def init_action_client(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction) #Creates a simple action client for the move_base node. 
        self.client.wait_for_server() #Waits for a response
        
    def coordinate_callback_thread(self, x, y, z):
        pose=Pose() #Empty pose object
        
        #Sets the position values to the inputs. 
        pose.position.x=x 
        pose.position.y=y
        pose.position.z=z
        pose.orientation.w=1.0 #Sets the orientation to be 1. 
        
        goal=MoveBaseGoal() #Creates a goal object
        #Sets up the frame for the goal.
        goal.target_pose.header.frame_id='base_footprint'
        #Sets the target position. 
        goal.target_pose.pose=pose
        
        self.client.send_goal(goal) #Sends the goal to the robot
        self.client.wait_for_result() #Waits for a result. 

#Implementation of the above class. 
if __name__ == "__main__":
    client = MoveBaseClient() #Creates a new action client
    client.init_action_client() #Initializes the client
    #Gets the desired change in x and y position. 
    x = float(input("Enter a desired change in the robot's x position: "))
    y = float(input("Enter a desired change in the robot's y position: "))
    client.coordinate_callback_thread(x, y, 0.0) #Sends the updated target to the robot. 
