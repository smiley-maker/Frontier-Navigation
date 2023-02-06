#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
import tf2_ros
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd


def x_grow(x, y, grid, q, L=1, limit=3):
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

        if 0 <= x < len(grid) and 0 <= y < len(grid):
            grid[x][y] = 100
        
        grid = y_grow(x, y, grid, 1, R=1)
        grid = y_grow(x, y, grid, -1, R=1)
        grid = x_grow(x, y, grid, q, L=L, limit=limit)
    return grid

def y_grow(x, y, grid, p, R=1, limit=3):
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
        
        grid = y_grow(x, y, grid, p, R=R, limit=limit)
    
    return grid

def grow(grid, points):
    """
    Uses the x_grow method to completely grow the grid in two directions based on a list of desired points. 

    Args:
        grid (2D numpy array): the original occupancy list converted into a two dimensional grid
        points (2d list): list of lists with x,y coordinates in the 2D grid. 

    Returns:
        2D numpy array: Returns the completely updated grid. 
    """        
    for p in points:
        grid = x_grow(p[1], p[0], grid, 1, L=1)
        grid = x_grow(p[1], p[0], grid, -1, L=1)
    
    return grid

def getObjects(grid):
    points = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 100:
                points.append([j, i])
    return points

def getPointArray(points, occupancy_grid):
    """
    Creates Point objects from a list of lists giving [x,y] coordinates .

    Args:
        points (2D list): 2D list of points in the form [[x,y], [x,y]] 

    Returns:
        List of Point objects: Returns the list of points converted into Point objects (with a z value of 0). 
    """        
    pointArray = []

    for point in points:
        if type(point) == Point():
            x = (point.x*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.x
            y = (point.y*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.y                
        else:
            x = (point[0]*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.x
            y = (point[1]*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.y
        if len(point) == 3:
            z = point[2]
        else:
            z = 0

        p = Point(x, y, 0)
        pointArray.append(p)
    
    return pointArray

def formatGrid(occupancy_grid):
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

def calculateDistance(currentPos, goal):
    """
    Calculates the distance between two point objects

    Args:
        currentPos (Point): current position of the robot
        goal (Point): position of the centroid (goal)

    Returns:
        float: euclidean distance between points
    """    
    if type(goal) == list: goal = goal[0]
    dx = currentPos.x - goal.x
    dy = currentPos.y - goal.y
    dist = (dx**2 + dy**2)**(0.5)
    return dist

def createMarkers(points, indx, action, ns, color=[220, 150, 225], scale=0.05, style=8):
    """
    Creates a Marker object based on the given points

    Args:
        points (List of Point objects): Gives the x,y coordinates of the markers. 

    Returns:
        Marker: Returns a new Marker object with all points
    """        
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "map"
    marker.ns = ns
    marker.id = indx
    marker.action = action
    marker.type = style
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

def getRGB(hexValue):
    """
    Helper function to convert a hex color to RGB. Every two digits in a hex number represent one color
    in rgb. The hex value can be retrieved using the int() method. 

    Args:
        hex (_type_): String based hex color (e.g. #fcba03)

    Returns:
        List: List containing three r, g, and b values
    """        
    rgbList = []
    for i in (0,2,4):
        decimal = int(hexValue[i:i+2], 16)
        rgbList.append(decimal)
    return rgbList

def findFrontiers(grid):
    """
    Finds all of the frontiers (places where -1 and 0 meet) and converts it into a new grid in which frontiers are given by 100

    Args:
        grid (2D Numpy Array): Reshaped occupancy array containing information about the current map based on what the robot can see. 

    Returns:
        2D Numpy Array: Returns a new binary array containing 100's where frontiers are found and 0's elsewhere. 
    """
    w, h = grid.shape
    updatedGrid = np.zeros((w,h), int)
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