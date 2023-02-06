#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np
import cv2

class frontierDetection():
    """
    Class to perform frontier detection based on data returned from the rviz map. 
    """    

    curr_grid = [] #Class variable to store the current occupancy data in a grid format
    def __init__(self):
        """
        Initializes the frontierDetection class by setting up a node and subscriber. 
        """        
        self.init_node()
        self.init_subscriber()
    
    def init_node(self):
        """
        Creates a new node with the name frontiers to support subscribing and publishing. 
        """        
        rospy.init_node("frontiers")
    
    def init_subscriber(self):
        """
        Initializes a map subscriber to obtain information about the occupancy of the robot's environment. 
        Connects to a callback called publish_markers. 
        """        
        sub = rospy.Subscriber('map', OccupancyGrid, self.publish_markers)
        rospy.spin()
    
    def init_publisher(self):
        """
        Initializes a publisher object using rospy in order to publish markers
        showing how the grid was expanded on the visualization_marker topic. 
        The latch is set to true in order to save the previous map

        Returns:
            Publisher: Returns the rospy publisher. 
        """
        return rospy.Publisher('/expansion', Marker, queue_size=1, latch=True)
    
    def x_grow(self, x, y, grid, q, L=1, limit=3):
        """
        Given a 2D grid, this recursive function will expand the grid by moving along the first row in the
        x direction and, no each iteration, going down the current column and expanding each cell by the
        limit size. 

        Args:
            x (integer): x position in grid (column)
            y (integer): y position in grid (row)
            grid (2D numpy array): the original occupancy list converted into a two dimensional grid. 
            q (integer): Gives a simple directionality for motion in the x direction, e.g. -1 or 1
            L (int, optional): Incremental variable to determine when the limit has been reach by the recursion. Defaults to 1 as starting value.
            limit (int, optional): Gives the amount of growth in the x direction. Defaults to 3 because that is approximately half
            of the size of the robot within the frame of this grid.

        Returns:
            2D Numpy Array: returns the updated grid in which the obstacles have been padded based on the limit. 
        """        
        if L < limit:
            L += 1
            x += q
            #Border detection
            if 0 <= x < len(grid) and 0 <= y < len(grid):
                grid[x][y] = 100
            
            #Moves both in the negative and positive y direction from the current x position
            grid = self.y_grow(x, y, grid, 1, R=1)
            grid = self.y_grow(x, y, grid, -1, R=1)
            #Makes a recursive call to the x_grow function to move to the next x position
            grid = self.x_grow(x, y, grid, q, L=L, limit=limit)
        
        return grid
    
    def y_grow(self, x, y, grid, p, R=1, limit=3):
        """
        Similar to x_grow, this function is used to grow a given column. It uses the same limit by default to account
        for the robot's size in the y direction as well. 

        Args:
            x (integer): x position in grid
            y (integer): y position in grid
            grid (2D numpy array): the original occupancy list converted into a two dimensional grid. 
            p (integer): Gives the directionality for growth in the y direction (-1 or 1 for up or down)
            R (int, optional): Incremental variable to determine when the limit has been reached by the recursion. Defaults to 1.
            limit (int, optional): Gives the amount of growth in the x direction. Defaults to 3 because that is approximately half of the
            robot's length.

        Returns:
            2D numpy array: updated grid in which the obstacles have been padded by the limit
        """        
        if R < limit:
            R += 1
            y += p

            if 0 <= x < len(grid) and 0 <= y < len(grid):
                grid[x][y] = 100
            
            grid = self.y_grow(x, y, grid, p, R=R, limit=limit)
        
        return grid
    
    def grow(self, grid, points):
        """
        Uses the x_grow method to completely grow the grid in two directions based on a list of desired points. 

        Args:
            grid (2D numpy array): the original occupancy list converted into a two dimensional grid
            points (2d list): list of lists with x,y coordinates in the 2D grid. 

        Returns:
            2D numpy array: Returns the completely updated grid. 
        """        
        for p in points:
            grid = self.x_grow(p[1], p[0], grid, 1, L=1)
            grid = self.x_grow(p[1], p[0], grid, -1, L=1)
        
        return grid
    
    def findFrontiers(self, grid):
        """
        Finds all of the frontiers (places where -1 and 0 meet) and converts it into a new grid in which frontiers are given by 100

        Args:
            grid (2D Numpy Array): Reshaped occupancy array containing information about the current map based on what the robot can see. 

        Returns:
            _type_: Returns a new binary array containing 100's where frontiers are found and 0's elsewhere. 
        """
 
        w, h = grid.shape
        updatedGrid = np.zeros(grid.shape, int)
        for i in range(w-1):
            for j in range(h-1):
                if grid[i][j] == -1:
                    if grid[i+1][j] == 0:
                        updatedGrid[i][j] = 100
                    elif grid[i-1][j] == 0:
                        updatedGrid[i][j] = 100
                    elif grid[i][j+1] == 0:
                        updatedGrid[i][j] = 100
                    elif grid[i][j-1] == 0:
                        updatedGrid[i][j] = 100
                    elif grid[i+1][j+1] == 0:
                        updatedGrid[i][j] = 100
                    elif grid[i+1][j-1] == 0:
                        updatedGrid[i][j] = 100
                    elif grid[i-1][j+1] == 0:
                        updatedGrid[i][j] = 100
                    elif grid[i-1][j-1] == 0:
                        updatedGrid[i][j] = 100
        return updatedGrid


    def visualizeFrontiers(self, frontiersGrid, header, info):
        """
        Creates a publisher object that publishes an occupancy grid to the /frontiers_map topic. 

        Args:
            frontiersGrid (2D numpy array): a grid representing the position of all of the frontiers
            header (_type_): dictionary object containing the relevant info to publish the grid
            info (_type_): dictionary object containing the important info to publish the grid. 
        """        
        grid_pub = rospy.Publisher("/frontiers_map", OccupancyGrid, queue_size=1, latch=True)
        occGrid = OccupancyGrid(header, info, frontiersGrid.flatten(order="C"))
        grid_pub.publish(occGrid)

    
    def getObjects(self, grid):
        """
        Searches through a 2D numpy array to find all instances of 100 (giving an object in the real map)

        Args:
            grid (2D numpy array): Grid representing the robot's environment using numerical values. 

        Returns:
            2D list: returns a list of points (x,y list) giving the objects
        """        
        points = []
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 100:
                    points.append([j, i])
        return points
    
    def getPointArray(self, points, occupancy_grid):
        """
        Creates Point objects from a list of lists giving [x,y] coordinates .

        Args:
            points (2D list): 2D list of points in the form [[x,y], [x,y]] 

        Returns:
            List of Point objects: Returns the list of points converted into Point objects (with a z value of 0). 
        """        
        pointArray = []

        for point in points:

            x = (point[0]*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.x
            y = (point[1]*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.y

            p = Point(x, y, 0)
            pointArray.append(p)
        
        return pointArray
    
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
    
    def compareGrid(self, currentGrid, newGrid):
        """
        Helper function to compare two 2D grids. Returns True if they are different. 

        Args:
            currentGrid (2D Numpy array): The grid from a previous stored instance
            newGrid (2D Numpy array): The new, incoming, grid. 

        Returns:
            Boolean: True or false based on whether the two grids were equal or not. 
        """
        if sum(currentGrid) == sum(newGrid):
            return False
        return True
    
    def createMarkers(self, points):
        """
        Creates a Marker object based on the given points

        Args:
            points (List of Point objects): Gives the x,y coordinates of the markers. 

        Returns:
            _type_: Returns a new Marker object with all points
        """        
        marker = Marker()
        scale = 0.03

        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = "objects"
        marker.type = 8
        marker.id = 0
        marker.color.r = 1.0
        marker.color.g = round(61/255, 2)
        marker.color.b = round(223/255, 2)
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
    
    def publish_markers(self, occupancy_grid):
        #Creates a new grid object from the input tuple data. 
        grid = self.formatGrid(occupancy_grid)

        #If the new grid is different than the current instance,
        if self.compareGrid(frontierDetection.curr_grid, occupancy_grid.data):
            
            #Updates the current grid to the new grid
            frontierDetection.curr_grid = occupancy_grid.data

            #Creates a publisher object
            pub = self.init_publisher()

            #Creates a two-dimensional array
#            grid = self.formatGrid(occupancy_grid)

            #Obtains all of the locations where 100 occurs
            locs = self.getObjects(grid)

            #Modifies the grid by expanding the area around the walls
            grid = self.grow(grid, locs)

            #Obtains all of the new locations where 100 occurs
            locs = self.getObjects(grid)

            #Converts the locations into point objects
            points = self.getPointArray(locs, occupancy_grid)

            #Places markers at all of the occupied spaces
            markers = self.createMarkers(points)

            #Publishes the markers
            pub.publish(markers)

            #Getting the frontiers grid:
            frontiersGrid = self.findFrontiers(grid)
#            print(np.count_nonzero(frontiersGrid)) Uncomment to see the current number of clusters

            #Getting data from original occupancy grid
            header = occupancy_grid.header
            info = occupancy_grid.info

            #Visualizing the frontiers
            self.visualizeFrontiers(frontiersGrid, header, info)
        
        else:
            print("There was no recorded change in the grid. ")


if __name__ == "__main__":
    detector = frontierDetection()
