#!/usr/bin/env python3

#Imports
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
import tf2_ros
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf2_geometry_msgs import PoseStamped, do_transform_pose
import actionlib
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
import sys
import math
from actionlib_msgs.msg import GoalStatus
from util import *
import random


class frontierEvaluation():
    #Class variables used throughout
    cache = pd.DataFrame({"x": [], "y": [], "cluster": [], "centroid_x": [], "centroid_y": [], "distance": []})
    occ_grid = None
    curr_grid = []
    tfBuffer = tf2_ros.Buffer()
    pos = Pose()
    locs = None
    frontiersGrid = None
    frontier_markers = None
    centroids = None
    goalPos = None
    grid = None
    
    def __init__(self):
        """
        Constructor setting up a node, action client, tf listener, publishers, and subscribers. 
        It also calls the main function. Note that I decided to have a main program running
        simultaneously to the subscriber callback to improve performance. 
        """        
        self.init_node()
        self.init_action_client()
        self.init_listener()
        self.init_publishers()
        self.init_subscriber()
        self.main_program()
    
    def init_node(self):
        """
        Initializes a node called explorer to support frontier exploration. 
        """        
        rospy.init_node("explorer")
    
    def init_subscriber(self):
        """
        Initializes a subscriber using rospy that takes in the map as an occupancy grid. 
        """
        sub = rospy.Subscriber("map", OccupancyGrid, self.callback)
        #rospy.spin()
    
    def init_publishers(self):
        """
        Initializes several publishers for different tasks throughout the file. 
        """        
        #Publishes markers indicating obstacle growing.
        self.pub = rospy.Publisher("/expansion", Marker, queue_size=1, latch=True)
        #Publishes an occupancy grid to show the frontiers
        self.grid_pub = rospy.Publisher("/frontiers_map", OccupancyGrid, queue_size=1, latch=True)
        #Publishes a marker array showing frontier clusters
        self.frontiers_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1, latch=True)
        #Publishes markers indicating the centroids of the frontier clusters. 
        self.centroids_pub = rospy.Publisher("/centroids", Marker, queue_size=1, latch=True)
    
    def init_listener(self):
        """
        Creates a tf listener and obtains the transformation data using the tfBuffer class variable. 
        """
        while not rospy.is_shutdown() :        
            try:
                listener = tf2_ros.TransformListener(frontierEvaluation.tfBuffer)
                trans = frontierEvaluation.tfBuffer.lookup_transform("map", "base_footprint", rospy.Time())
                x,y = trans.transform.translation.x,trans.transform.translation.y
                frontierEvaluation.pos.position.x = x
                frontierEvaluation.pos.position.y = y
                break
            except:
                rospy.Rate(10.0).sleep()
                continue
    
    def init_action_client(self):
        """
        Creates an action client to send goals to the robot using a move base action. 
        """        
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
    
    def coordinate_callback(self, x, y, z):
        """
        Creates a move base goal using a Pose Stamped object. It uses the base footprint
        id to transform the map's coordinates into the robot's coordinates in order to 
        move the base relative to the robot's world view. It sends the goal in terms of how 
        much the robot needs to move from its current position. 

        Args:
            x (float): x position of the goal in the map frame
            y (float): y position of the goal in the map frame
            z (float): z position of the goal in the map frame
        """        

        pose = Pose() #Creates an empty pose object for the input coordinates in the map frame.

        #Sets the coordinates of the pose object
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        #Sets up a listener for the transformation of map to robot coordinates
        listener = tf2_ros.TransformListener(frontierEvaluation.tfBuffer)
        while not rospy.is_shutdown():
            rospy.sleep(0)
            try:
                #looks up the transformation to the robot's frame.
                trans = frontierEvaluation.tfBuffer.lookup_transform("base_footprint", "map", rospy.Time(0))
                break
            except:
                continue
        
        #The move base action client requires a pose stamped object to setup
        #the goal
        tfPose = PoseStamped()
        tfPose.pose = pose
        tfPose.header.frame_id = "base_footprint"
        tfPose.header.stamp = rospy.Time.now()
        outPose = do_transform_pose(tfPose, trans)

        goal = MoveBaseGoal() #Sets up a move base goal to send to the robot
        goal.target_pose.header.frame_id = "base_footprint"
        goal.target_pose.pose=outPose.pose
        try:
            self.client.send_goal(goal) #Tries to send the goal
        except:
            rospy.sleep(0)

    def dbscan(self, frontiers, epsilon, min_points):
        """
        This method uses scikit learns dbscan to find all of the continuous frontiers in the given list. 
        It finds all clusters that meet the minimum number of points, and classifies the rest as noise. 
        I created a dataframe with the x,y, and cluster value for each point to make it easier to use. 

        Args:
            frontiers (2D list): List of the points where the frontiers are located
            epsilon (float): the maximum distance between two points to be considered in the same cluster. 
            min_points (int):The minimum number of samples required for a cluster
        
        Returns:
            Pandas Dataframe: dataframe containing each x,y location and which cluster the points belong to. 
        """        
        frontiers = pd.DataFrame(frontiers)
        db = DBSCAN(eps=epsilon, min_samples=min_points, metric="l1")
        clusters = db.fit_predict(frontiers)
        frontiers["cluster"] = clusters
        frontiers = frontiers[frontiers["cluster"] != -1]
        frontiers = frontiers.sort_values('cluster')
        return frontiers
    
    def visualizeFrontiers(self, frontiersGrid, header, info):
        """
        Creates a publisher object that publishes an occupancy grid to the /frontiers_map topic. 

        Args:
            frontiersGrid (2D numpy array): a grid representing the position of all of the frontiers
            header (_type_): dictionary object containing the relevant info to publish the grid
            info (_type_): dictionary object containing the important info to publish the grid. 
        """
        occGrid = OccupancyGrid(header, info, frontiersGrid.flatten(order="C"))
        self.grid_pub.publish(occGrid)
    
    def colorClusters(self, df, occupancy_grid):
        """
        This function is responsible for formatting the dataframe based cache as
        well as giving each frontier cluster a color. Centroids are calculated, 
        as well as distances. 

        Args:
            df (pandas DataFrame): dataset containing x (0), y (1), and cluster columns.
            occupancy_grid (OccupancyGrid): object including the header, info, and data sections of occupancy grid. 

        Returns:
            _type_: _description_
        """        
        clusters = list(set(df["cluster"]))
        markers = MarkerArray()
        centers = []
        palette = list(reversed(sns.color_palette("rainbow", len(clusters)).as_hex()))
        colors = []
        distances = []
        centroid_x = []
        centroid_y = []
        self.init_listener()
        for p in palette:
            p = p.replace("#", "")
            colors.append(getRGB(p))
        
        for c in range(len(clusters)):
            sub_df = df[df["cluster"] == clusters[c]]

            points = getPointArray([(x,y) for x,y in zip(sub_df[0], sub_df[1])], occupancy_grid)
            marker = createMarkers(points=points, indx=c, action=0, ns="frontiers", color=colors[c], scale=0.1, style=7)
            markers.markers.append(marker)

            xc = int(sum(sub_df[0])/len(sub_df[0]))
            yc = int(sum(sub_df[1])/len(sub_df[1]))
            centroid_x += [xc]*len(sub_df)
            centroid_y += [yc]*len(sub_df)
            center = [xc, yc, 20]
            centers.append(center)
            centerPoint = getPointArray([center], occupancy_grid)
            self.init_listener()
            distance = [calculateDistance(frontierEvaluation.pos.position, centerPoint)]*len(sub_df)
            distances += distance

        centers = getPointArray(centers, occupancy_grid)
        centerMarker = createMarkers(points=centers, indx=c, action=0, ns="centroids", color=[255,255,255], scale=0.3, style=7)
        
        df["distance"] = distances
        df["centroid_x"] = centroid_x
        df["centroid_y"] = centroid_y
        df.sort_values("distance" , inplace = True, ascending=True)
        df.reset_index(inplace=True)

        return markers, centerMarker, df
    
    def compareGrid(self, currentGrid, newGrid):
        """
        Helper function to check if two flattened grids are equal. 

        Args:
            currentGrid (_type_): Data containing numeric values concerning occupancy
            newGrid (_type_): Updated occupancy data

        Returns:
            Boolean: True if grids aren't equal. 
        """        
        if sum(currentGrid) == sum(newGrid):
            return False
        return True
    
    
    def habitability(self, x, y, grid):
        """
        This function determines if an (x,y) point on a 2D grid is habitable
        (i.e. open space). Walls or obstacles are represented by 100 in the grid.

        Args:
            x (int): Current x position in grid (0 - 384)
            y (int): Current y position in grid (0 - 384)
            grid (2D Numpy array): Grid containing occupancy information

        Returns:
            Boolean: returns true if the current position is habitable
        """        
        for i in range(3):
            if grid[x+i][y] == 100:
                return False
            if grid[x-i][y] == 100:
                return False
            if grid[x][y+i] == 100:
                return False
            if grid[x][y-i] == 100:
                return False
        return True
    
    def findPoint(self, x, y, grid):
        """
        If the current point is not habitable, this function will determine a new
        position for the current point (used to move centroids to places the robot
        can feasibly visit).

        Args:
            x (float): current map x position
            y (float): current map y position
            grid (2D Numpy Array): 2D Numpy array containing occupancy information. 

        Returns:
            Tuple: Returns a tuple containing the updated (x,y) point
        """        
        for i in range(15):
            dx = random.randint(-5, 5)
            dy = random.randint(-5, 5)
            if self.habitability(x+dx, y+dy, grid):
                return (x+dx, y+dy)
        return None
    
    def main_program(self):
        """
        This function operates alongside the subscriber callback to calculate 
        goal positions, publish markers, and enact search functionality, and 
        obtain results. 
        """        
        #This is just a check to make sure that the subscriber callback has occurred before running
        while type(frontierEvaluation.frontiersGrid) == type(None) and not rospy.is_shutdown():
            rospy.sleep(1)
        
        #Runs until all frontiers have been visited. 
        while len(set(frontierEvaluation.cache["cluster"])) > 0  and not rospy.is_shutdown():
            rospy.sleep(1)

            #Gets a list of the unique frontiers (the coordinates, cluster, and distance from the robot)
            goals = list(set([(x,y,c,d) for x,y,c,d in zip(frontierEvaluation.cache["centroid_x"], frontierEvaluation.cache["centroid_y"], frontierEvaluation.cache["cluster"], frontierEvaluation.cache["distance"])]))
            #Sorts the list by the distance from the robot
            goals = sorted(goals , key = lambda x : x[3], reverse=False)            
            #Gets the closest goal
            goalPoint = (goals[0][0], goals[0][1], 0) 
            cluster = goals[0][2] #Gets the goal cluster

            #Publishing updated obstacles
            points = getPointArray(frontierEvaluation.locs, frontierEvaluation.occ_grid)
            markers = createMarkers(points=points, indx=0, action=0, ns="objects", scale=0.05)

            #Sending goal:
            print("sending goal...")
            #Checks the habitability of the current centroid
            if self.habitability(goalPoint[0], goalPoint[1], frontierEvaluation.grid):
                #If the centroid is habitable, calculate the goal point as:
                goalPoint = getPointArray([goalPoint], frontierEvaluation.occ_grid)[0]
            else:
                #Find a habitable point
                x, y = self.findPoint(goalPoint[0], goalPoint[1], frontierEvaluation.grid)
                if x and y: #If point found,
                    #Set the goal to:
                    goalPoint = getPointArray([(x,y,0)], frontierEvaluation.occ_grid)[0]
                else:
                    print("Goal is unreachable. ")
                    frontierEvaluation.cache = frontierEvaluation.cache[frontierEvaluation.cache["cluster"] != cluster]
                    self.client.cancel_all_goals()
                    break
            
            self.coordinate_callback(goalPoint.x, goalPoint.y, goalPoint.z)
            goalMarker = createMarkers(points=[goalPoint], indx=0, action=0, ns="goal", color=[255, 30, 200], scale=0.3, style=7)

            #Waiting for result
            result = None
            rate = rospy.Rate(1.0)
            while result is None and not rospy.is_shutdown() :
                #Deleting:
                m = Marker(action=3)
                self.pub.publish(m)
                self.centroids_pub.publish(m)
                mrkrArr = MarkerArray()
                mrkrArr.markers.append(m)
                self.frontiers_pub.publish(mrkrArr)

                #Visualizing
                self.pub.publish(markers)
                self.visualizeFrontiers(frontierEvaluation.frontiersGrid, frontierEvaluation.occ_grid.header, frontierEvaluation.occ_grid.info)
                self.frontiers_pub.publish(frontierEvaluation.frontier_markers)
                self.centroids_pub.publish(frontierEvaluation.centroids)
                self.centroids_pub.publish(goalMarker)

                x,y = goalPoint.x, goalPoint.y

                self.init_listener() #Gets robot's position
                if ((frontierEvaluation.pos.position.x - x)**2 + (frontierEvaluation.pos.position.y - y)**2)**0.5 < 0.3:
                    #Updates dataframe
                    frontierEvaluation.cache = frontierEvaluation.cache[frontierEvaluation.cache["cluster"] != cluster]
                    #Cancels goal
                    self.client.cancel_all_goals()
                    break

                #Getting result
                result = self.client.get_result()

                if self.client.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal found!")
                    frontierEvaluation.cache = frontierEvaluation.cache[frontierEvaluation.cache["cluster"] != cluster]
                    self.client.cancel_all_goals()
                    break

                
                if self.client.get_state() == GoalStatus.ABORTED:
                    rospy.loginfo("Can't find goal. Moving to the next frontier.")
                    frontierEvaluation.cache = frontierEvaluation.cache[frontierEvaluation.cache["cluster"] != cluster]
                    self.client.cancel_all_goals()
                    break

                rate.sleep()
            

        if rospy.is_shutdown():
            print("Code stopping...")
            sys.exit("Rospy shut down.")
        
        elif len(set(frontierEvaluation.cache["cluster"])) == 0:
            print("Congrats! All centroids were explored! Returning to the origin....")
            self.coordinate_callback_thread(0, 0, 0)
            self.client.wait_for_result()
            sys.exit("Exiting the program. ")
    
    def callback(self, occupancy_grid):
        
        frontierEvaluation.occ_grid = occupancy_grid
        if self.compareGrid(frontierEvaluation.curr_grid, occupancy_grid.data):
            frontierEvaluation.curr_grid = occupancy_grid.data

            print("setting up")
            frontierEvaluation.grid = formatGrid(occupancy_grid)

            locs = getObjects(frontierEvaluation.grid)
            frontierEvaluation.grid = grow(frontierEvaluation.grid, locs)
            frontierEvaluation.locs = getObjects(frontierEvaluation.grid)

            frontierEvaluation.frontiersGrid = findFrontiers(frontierEvaluation.grid)

            frontiers = getObjects(frontierEvaluation.frontiersGrid)
            df = self.dbscan(frontiers, 5, 8)

            frontierEvaluation.frontier_markers, frontierEvaluation.centroids, frontierEvaluation.cache = self.colorClusters(df, frontierEvaluation.occ_grid)

            print("done")

        else:
            print("There was no recorded change")

if __name__ == "__main__":
    f = frontierEvaluation()