#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 18 15:00:00 2022

@author: pulkit
"""

import numpy as np
import cv2
import math
from Node import Node
from queue import PriorityQueue

def line(p1, p2, x, y, t):
    """
    Constructs a line passing through two points and calculates the distance of a given point from the line. 

    Parameters
    ----------
    p1 : Array
        Coordinates of point 1.
    p2 : Array
        Coordinates of point 2.
    x : double
        x-coordinate of point of interest.
    y : double
        y-coordinate of point of interest..
    t : double
        offset from line for provided clearance.

    Returns
    -------
    d : double
        Distance of point from line along with the sign indicating direction.

    """
    m = (p2[1] - p1[1]) / ( p2[0] - p1[0])
    d = m * (x - p1[0]) + (p1[1] + (t * math.sqrt((m ** 2) + 1)) - y)
    
    return d


def circle(center, r, x, y, t):
    """
    Constructs a circle for a given center point and radius and calculates the distance of a given point from the line.

    Parameters
    ----------
    center : Array
        Coordinates of the center of the circle..
    r : double
        Radius of the circle.
    x : double
        x-coordinate of point of interest..
    y : double
        y-coordinate of point of interest..
    t : double
        offset from the circle for provided clearance.

    Returns
    -------
    d : double
        Distance of point from line along with the sign indicating direction.

    """
    d = (x - center[0])**2 + (y - center[1])**2 - (r + t)**2
    
    return d


def isObstacle(point):
    """
    Checks whether the point collides with an obstacle.

    Parameters
    ----------
    point : Array
        Point coordinates.
    map_ : 2D Array
        Constructed map_.

    Returns
    -------
    flag : bool
        True if point coincides with an obstacle, false otherwise.

    """
    
    flag = False
    
    # if map_[point[1],point[0]] == 1:
    #     flag = True
    
    # return flag
    map_ = np.zeros((250,400))
    
    r = 0                                       # radius
    c = 5                                       # clearance
    t = r + c                                   # Total clearance
    i = point[1]
    j = point[0]
    # Circular Obstacle, 
    if (circle((300,65),40,i,j,t) < 0):
        flag = True
   
    # Arrow Obstacle    
    if (line((36,65),(115,40),i,j,t) < 0 and line((36,65),(105,150),i,j,t) > 0 
        and line((80,70),(105,150),i,j,t) < 0):
        flag = True
    if (line((80,70),(105,150),i,j,t) > 0 and line((36,65),(115,40),i,j,t) < 0 
        and line((80,70),(115,40),i,j,t) > 0):
        flag = True
    
    # Hexagonal Obastacle
    if (i > (165-t) and i < (235+t) 
        and line((165,129.79),(200,109.59),i,j,t) < 0 
        and line((200,109.59),(235,129.79),i,j,t) < 0 
        and line((165,170.20),(200,190.41),i,j,t) > 0 
        and line((200,190.41),(235,170.20),i,j,t) > 0):
        flag = True
    
    # Boundaries of the map_
    if (i > 0 and i < t):
        flag = True
    if (i < 400 and i > (400-t)):
        flag = True
    if (j > 0 and j < t):
        flag = True
    if (j < 250 and j >(250 - t)):
        flag = True
    
    return flag


def create_map():
    """
    Creates the given map_ in a numpy array.

    Returns
    -------
    map_ : 2D Array
        Created map_.

    """
    map_ = np.zeros((250,400))
     
    # Clearance is also added in the same color code, so the obstacles are appear to be inflated.
   
    for i in range(map_.shape[1]):
        for j in range(map_.shape[0]):
            if isObstacle((j,i)):
                map_[j][i] = 1    
    return map_


def getStartNode(map_):
    """
    Gets the start node from the user.

    Returns
    -------
    start_node : Array
        Coordinates of start node.

    """
    
    flag = False
    while not flag:
        start_node = [int(item) for item in input("\n Please enter the start node: ").split(',')]
        start_node[1] = 250 - start_node[1]
        if (len(start_node) == 2 and (0 <= start_node[0] <= 400) and (0 <= start_node[1] <= 250)):
            if not isObstacle(start_node,map_):
                flag = True
            else:   
                print("Start node collides with obstacle \n")
        else:
            print("The input node location does not exist in the map_, please enter a valid start node.\n")
            flag = False
    orientation = int(input("\n Please enter initial orientation: "))
    if (orientation > 360):
        orientaion = orientation % 360   
      
    return start_node, orientation


def getGoalNode(map_):
    """
    Gets the goal node from the user.

    Returns
    -------
    goal_node : Array
        Coordinates of goal node.

    """
    
    flag = False
    while not flag:
        goal_node = [int(item) for item in input("\n Please enter the goal node: ").split(',')]
        goal_node[1] = 250 - goal_node[1]
        if (len(goal_node) == 2 and (0 <= goal_node[0] <= 400) and (0 <= goal_node[1] <= 250)):
            if not isObstacle(goal_node,map_):
                flag = True
            else:
                print("Goal node collides with obstacle \n")
        else:
            print("The input node location does not exist in the map_, please enter a valid goal node.\n")
            flag = False
    
    orientation = int(input("\n Please enter initial orientation: "))
    if (orientation > 360):
        orientaion = orientation % 360  
        
    return goal_node, orientation

def getData():
    
    clearance, radius = [int(item) for item in input("\n Please enter the clearance and robot radius (comma seperated values): ").split(',')]
    flag = False
    while not flag:     
        StepSize = int(input("\n Please enter the step size: "))
        if (1 <= StepSize <= 10):
            flag = True
        else:
            print("\n Enter step size in range [1,10].")
    angle = int(input("\n Please enter angle between movements: "))
    
    return clearance, radius, StepSize, angle


def Euclidean(current, goal):
    cost=0.0
    if current is not None:
        cost=np.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)
    
    return cost

def explore(node, L, angle):
    """
    Explores the neighbors of the current node and performs move action.

    Parameters
    ----------
    node : Node
        Current node for which neighbors need to be explored.
    map : 2D Array
        Constructed map.

    Returns
    -------
    valid_paths : list
        Action performed node, cost of movement.

    """
    x = node.x
    y = node.y
    angle = node.angle

    moves = [ActionMoveCC60(node, L, angle),
             ActionMoveCC30(node, L, angle),
             ActionMoveStraight(node, L, angle),
             ActionMoveC30(node, L, angle),
             ActionMoveC60(node, L, angle)]
    
    valid_paths = []
    for move in moves:
        if not isObstacle(tuple(move[0])):
            valid_paths.append([move[0],move[1]])

    return valid_paths

def ActionMoveCC60(node, L, angle):
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = cur_angle + 2 * angle
    
    new_x = x + L * np.cos(np.radians(new_angle))
    new_y = y + L * np.sin(np.radians(new_angle))
    
    move = (new_x, new_y)
    cost = L 
    
    return [move,cost]


def ActionMoveCC30(node, L, angle):
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = cur_angle + angle
    
    new_x = x + L * np.cos(np.radians(new_angle))
    new_y = y + L * np.sin(np.radians(new_angle))
    
    move = (new_x, new_y)
    cost = L 
    
    return [move,cost]    


def ActionMoveStraight(node, L, angle):
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = cur_angle 
    
    new_x = x + L * np.cos(np.radians(new_angle))
    new_y = y + L * np.sin(np.radians(new_angle))
    
    move = (new_x, new_y)
    cost = L 
    
    return [move,cost]   
  
def ActionMoveC30(node, L, angle):
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = cur_angle - angle
    
    new_x = x + L * np.cos(np.radians(new_angle))
    new_y = y + L * np.sin(np.radians(new_angle))
    
    move = (new_x, new_y)
    cost = L 
    
    return [move,cost]  
 
def ActionMoveC60(node, L, angle):
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = cur_angle - 2 * angle
    
    new_x = x + L * np.cos(np.radians(new_angle))
    new_y = y + L * np.sin(np.radians(new_angle))
    
    move = (new_x, new_y)
    cost = L 
    
    return [move,cost]   

def isGoal(current, goal, threshold):
    
    distance = Euclidean(current, goal)
    
    if (distance < threshold):
        return True
    else:
        return False
  
    
def Dijkstra(start_node, goal_node, map):
    """
    Performs Djikstra's search.

    Parameters
    ----------
    start_node : Array
        Initial node.
    goal_node : Array
        Goal node.
    map : 2D Array
        Constructed map.

    Returns
    -------
    node_objects : dict
        Dictionary holding information of nodes, instances of class Node.

    """

    print("\n Performing Djikstra search...\n")

    q = PriorityQueue()                                                              # Priority queue for open nodes
    visited = set([])                                                                # Set conataining visited nodes
    node_objects = {}                                                                # dictionary of nodes
    distance = {}                                                                    # distance 
    
    # Assign costs for all nodes to a large value
    for i in range(0, map.shape[1]):
        for j in range(0, map.shape[0]):
            distance[str([i,j])] = 9999999
    
    distance[str(start_node)] = 0                                                    # Start node has cost of 0
    visited.add(str(start_node))                                                     # Add start node to visited list
    node = Node.Node(start_node,0,None)                                              # Create instance of Node
    node_objects[str(node.pos)] = node                                               # Assigning the node value in dictionary
    q.put([node.cost, node.pos])                                                     # Inserting the start node in priority queue

    while not q.empty():                                                             # Iterate until the queue is empty
        node_temp = q.get()                                                          # Pop node from queue
        node = node_objects[str(node_temp[1])]  
                                     
        # Check of the node is the goal node
        if node_temp[1][0] == goal_node[0] and node_temp[1][1] == goal_node[1]:      
            print(" Goal Reached!!!\n")
            node_objects[str(goal_node)] = Node.Node(goal_node,node_temp[0], node)
            break
        
        for next_node, cost in explore(node,map):                                    # Explore neighbors

            if str(next_node) in visited:                                            # Check if action performed next node is already visited
                cost_temp = cost + distance[str(node.pos)]                           # Cost to come
                if cost_temp < distance[str(next_node)]:                             # Update cost
                    distance[str(next_node)] = cost_temp
                    node_objects[str(next_node)].parent = node

            else:                                                                    # If next node is not visited
                visited.add(str(next_node))
                absolute_cost = cost + distance[str(node.pos)]
                distance[str(next_node)] = absolute_cost
                new_node = Node.Node(next_node, absolute_cost, node_objects[str(node.pos)])
                node_objects[str(next_node)] = new_node
                q.put([absolute_cost, new_node.pos])

    return node_objects

img = create_map()
cv2.imshow("image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()