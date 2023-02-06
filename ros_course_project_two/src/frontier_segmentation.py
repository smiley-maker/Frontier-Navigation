#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd


class frontierSegmentation():
    def __init__(self):
        """
        Constructor to initialize a node and subscriber
        """        
        self.init_node()
        self.init_subscriber()
    
    def init_node(self):
        """
        Creates a new rospy node called segments
        """  
        rospy.init_node('segments')
    
    def init_subscriber(self):
        """
        Creates a new subscriber that takes in the frontiers map as an occupancy grid from frontier_detection.py. 
        """        
        rospy.Subscriber("/frontiers_map", OccupancyGrid, self.callback)
        rospy.spin()
    
    def init_publisher(self):
        """
        Creates a publisher that publishes a marker array to the visualization_marker_array topic. 

        Returns:
            Publisher: rospy publisher
        """        
        return rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1, latch=True)
    
    def init_centroid_pub(self):
        """
        Creates a publisher that publishes markers to the centers topic

        Returns:
            Publisher: rospy publisher
        """        
        return rospy.Publisher("/centers", Marker, queue_size=1, latch=True)
    
    def getFrontiers(self, grid):
        """
        Searches through the frontiers map to find the frontiers (places where 100 is reported)

        Args:
            grid (2d nump array): takes in a 2D numpy grid

        Returns:
            2D list: List of points that give the location of each frontier
        """        
        frontiers = []

        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 100:
                    frontiers.append([j,i])
        
        return frontiers
    
    def formatGrid(self, occupancy_grid):
        """
        This method takes in a 1d occupancy grid and returns 
        a 2d matrix of occupancy values. 

        Args:
            occupancy_grid (OccupancyGrid): One dimensional tuple of occupancy values (in row major form). 

        Returns:
            2D Numpy Array: Numpy array containing all of the relevant occupancy values in the correct indeces. 
        """        
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        grid = np.reshape(occupancy_grid.data, (width, height))
        return grid
    
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
        frontiers = pd.DataFrame(frontiers) #Creates a dataframe using the list of (x,y) coordinates
        db = DBSCAN(eps=epsilon, min_samples=min_points, metric="l1") #Uses the l1 metric to find clusters
        clusters = db.fit_predict(frontiers) #Fits the DBSCAN object to the frontiers data
        frontiers["Cluster"] = clusters #Adds a column to the dataframe to indicate which cluster each point belongs to
        frontiers = frontiers[frontiers["Cluster"] != -1] #Removes the noise cluster
        frontiers = frontiers.sort_values('Cluster') #Sorts the dataframe by the cluster column
        return frontiers

    def getPointArray(self, points):
        """
        Creates Point objects from a list of lists giving [x,y] coordinates .

        Args:
            points (2D list): 2D list of points in the form [[x,y], [x,y]] 

        Returns:
            List of Point objects: Returns the list of points converted into Point objects (with a z value of 0). 
        """        
        pointArray = []
        if type(points) == list:
            for point in points:
                x = ((point[0]/384)*19.2)-10
                y = ((point[1]/384)*19.2)-10
                if len(point) == 3:
                    z = point[2]
                else:
                    z=0

                p = Point(x, y, z)
                pointArray.append(p)
        
        else:
            x = ((points[0]/384)*19.2)-10
            y = ((points[1]/384)*19.2)-10
            if len(points) == 3:
                z = points[2]
            else:
                z = 0
            p = Point(x, y, z)
            return p
        
        return pointArray
    
    def getRGB(self, hexValue):
        """
        Helper function to convert a hex color to RGB. Every two digits in a hex number represent one color
        in rgb. The hex value can be retrieved using the int() method. 

        Args:
            hex (_type_): String based hex color (e.g. #fcba03)

        Returns:
            List: List containing three r, g, and b values
        """        
        rgbList = []
        for i in (0, 2, 4):
            decimal = int(hexValue[i:i+2], 16)
            rgbList.append(decimal)
        
        return rgbList

    def createMarkers(self, indx, action, color=[220, 150, 255], points=[], ns="frontiers", scale=0.05, style=8):
        """
        Creates markers for a given points array. 

        Args:
            indx (int): index for the marker
            action (int): One of [0,1,2,3] representing which action to use (e.g. 0 - create, 3-delete all)
            color (list, optional): RGB color for the marker(s). Defaults to [220, 150, 255].
            points (list, optional): List of points to create marker. Defaults to [].
            ns (str, optional): Namespace for the marker(s). Defaults to "frontiers".
            scale (float, optional): Gives the size of the markers. Defaults to 0.05.
            style (int, optional): Type of marker. Defaults to 8 for points (7 gives spheres).

        Returns:
            Marker: Marker
        """        
        marker = Marker()

        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = ns
        marker.type = style
        marker.action = action
        marker.id = indx
        marker.color.r = round(color[0]/255, 2)
        marker.color.g = round(color[1]/255, 2)
        marker.color.b = round(color[2]/255, 2)
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration()
        marker.points = points
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker
    
    def colorClusters(self, df):
        """
        Gives each cluster in the dataframe a different color using a Seaborn color palette. 

        Args:
            df (Pandas Dataframe): dataframe containing x,y points and the cluster they belong to. 
        """ 
        clusters = list(set(df["Cluster"])) #Gets the unique clusters
        markers = MarkerArray() #Empty marker array
        centroids = [] #centroids list
        #Creates a palette using Seaborn's rainbow sequential palette. 
        palette = list(reversed(sns.color_palette("rainbow", len(clusters)).as_hex()))
        colors = [] #Empty list to store the RGB colors
        for p in palette: #Loops through the palette created. 
            p = p.replace("#", "") #Removes the # in each hex color
            colors.append(self.getRGB(p)) #Gets the RGB value using the helper function above. 

        for c in range(len(clusters)): #Loops through each unique cluster
            #Creates a smaller dataframe of the points belonging just to the current cluster. 
            sub_df = df[df["Cluster"] == clusters[c]] 
            #Changes the points to Point objects using list comprehension. 
            points = self.getPointArray([(x,y) for x,y in zip(sub_df[0], sub_df[1])])
            #Creates markers for each point
            marker = self.createMarkers(c, 0, color=colors[c], points=points, ns="frontiers", scale=0.1, style=7)
            #Appends the markers to the marker array
            markers.markers.append(marker)
            #Calculates the centroid of each cluster
            xc = sum(sub_df[0])/len(sub_df[0])
            yc = sum(sub_df[1])/len(sub_df[1])
            center = (xc,yc, 10) #Centroid point (note I used a z value of 10 here to make sure the centroids appeared on top)
            centroids.append(center) #Appends the center to the list of centroids
        centroids = self.getPointArray(centroids) #Creates points from the centroid locations
        #Creates a single marker from all of the centroid points. 
        centroidMarkers = self.createMarkers(c+len(clusters), 0, color=[255, 255, 255], points=centroids, ns="centers", scale=0.3, style=7)

        return markers, centroidMarkers
    
    
    def callback(self, occupancy_grid):
        """
        Callback function for the frontiers map subscriber. 

        Args:
            occupancy_grid (OccupancyGrid): occupancy data returned from the frontier_detection.py file. Includes expanded objects. 
        """        

        print("setting up")
        #Initializes both publishers for the clusters and centroids, respectively. 
        pub = self.init_publisher()
        cpub = self.init_centroid_pub()

        #Formats the occupancy grid into a 2D list. 
        grid = self.formatGrid(occupancy_grid)
        frontiers = self.getFrontiers(grid) #Gets frontiers from binary map
        
        #Gets a dataframe containing the clusters given the frontier points using dbscan. 
        #I chose 5 as the maximum distance for each frontier to be in the same cluster because it seemed to give reasonable clusters. 
        #I decided to have a minimum of 8 points for a cluster to be considered in order to remove outliers and unnecesarry groupings. 
        df = self.dbscan(frontiers, 5, 8)
        
        #Removes all of the markers
        m = Marker(action=3)
        cpub.publish(m)
        mrkrArr = MarkerArray()
        mrkrArr.markers.append(m)
        pub.publish(mrkrArr)

        #Gets the clusters and centroids from the dbscan dataframe
        markers, centroids = self.colorClusters(df)

        #Publishes the markers and centroids. 
        pub.publish(markers)
        cpub.publish(centroids)
        print("done")


if __name__ == "__main__":
    seg = frontierSegmentation()