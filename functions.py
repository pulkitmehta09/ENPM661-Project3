#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 18 15:00:00 2022

@author: pulkit
"""

import numpy as np
import cv2
import math
import Node
from queue import PriorityQueue
from cv2 import VideoWriter, VideoWriter_fourcc 


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


def isObstacle(point, c, r):
    """
    Checks whether the point collides with an obstacle.

    Parameters
    ----------
    point : Array
        Point coordinates.
    c: int
        Clearance.
    r: int
        Robot radius.    

    Returns
    -------
    flag : bool
        True if point coincides with an obstacle, false otherwise.

    """
    
    flag = False

    t = r + c                                   # Total clearance
    i = point[0]
    j = point[1]
    
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
    if (i > 0 and i < c):
        flag = True
    if (i < 400 and i > (400-c)):
        flag = True
    if (j > 0 and j < c):
        flag = True
    if (j < 250 and j >(250 - c)):
        flag = True
    
    return flag


def create_map(c, r):
    """
    Creates the given map in a numpy array.

    Parameters
    ----------
    c : int
        Clearance.
    r : int
        Robot radius.

    Returns
    -------
    map: 2D Array
        Created map.

    """
    map_ = np.zeros((250,400))
     
    # Clearance is also added in the same color code, so the obstacles are appear to be inflated.
   
    for i in range(map_.shape[1]):
        for j in range(map_.shape[0]):
            if isObstacle((i,j), c, r):
                map_[j][i] = 1    
    
    return map_


def getStartNode(c, r):
    """
    Gets the start node from the user.
    
    Parameters
    ----------
    c : int
        Clearance.
    r : int
        Robot radius.

    Returns
    -------
    start_node : Array
        Coordinates of start node.

    """
    
    flag = False
    while not flag:
        start_node = [int(item) for item in input("\n Please enter the start node: ").split(',')]
        start_node[1] = 250 - start_node[1]
        if (start_node[2] > 360):
            start_node[2] = start_node[2] % 360
        if (len(start_node) == 3 and (0 <= start_node[0] <= 400) and (0 <= start_node[1] <= 250)):
            if not isObstacle(start_node, c, r):
                flag = True
            else:   
                print("Start node collides with obstacle \n")
        else:
            print("The input node location does not exist in the map, please enter a valid start node.\n")
            flag = False
      
    return start_node


def getGoalNode(c, r):
    """
    Gets the goal node from the user.
    
    Parameters
    ----------
    c : int
        Clearance.
    r : int
        Robot radius.

    Returns
    -------
    goal_node : Array
        Coordinates of goal node.

    """
    
    flag = False
    while not flag:
        goal_node = [int(item) for item in input("\n Please enter the goal node: ").split(',')]
        goal_node[1] = 250 - goal_node[1]
        if (goal_node[2] > 360):
            goal_node[2] = goal_node[2] % 360
        if (len(goal_node) == 3 and (0 <= goal_node[0] <= 400) and (0 <= goal_node[1] <= 250)):
            if not isObstacle(goal_node, c, r):
                flag = True
            else:
                print("Goal node collides with obstacle \n")
        else:
            print("The input node location does not exist in the map, please enter a valid goal node.\n")
            flag = False
        
    return goal_node


def getData():
    """
    Takes the required parameter data from user.

    Returns
    -------
    clearance : int
        Clearance.
    radius : int
        Robot radius.
    StepSize : int
        Stride.
    angle : int
        Angle step-size.

    """
    
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
    """
    Calculates euclidean diatance between two nodes.

    Parameters
    ----------
    current : Array
        Source node.
    goal : Array
        Destination node.

    Returns
    -------
    cost : double
        Distance between the two nodes.

    """
    
    cost=0.0
    if current is not None:
        cost=np.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)
    
    return cost


def ActionMoveCC60(node, L, angle):
    """
    Performs action corresponding to angle of 60 degrees. 

    Parameters
    ----------
    node : Node
        Object of class Node.
    L : int
        Step-size(stride).
    angle : int
        Angle step-size.

    Returns
    -------
    list
        List containing new node, cost.

    """
    
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = (cur_angle + 2 * angle) % 360
    
    new_x = int(np.round(x + L * np.cos(np.radians(new_angle))))
    new_y = int(np.round(y + L * np.sin(np.radians(new_angle))))
    
    move = (new_x, new_y, new_angle)
    cost = L 
    
    return [move,cost]


def ActionMoveCC30(node, L, angle):
    """
    Performs action corresponding to angle of 30 degrees. 

    Parameters
    ----------
    node : Node
        Object of class Node.
    L : int
        Step-size(stride).
    angle : int
        Angle step-size.

    Returns
    -------
    list
        List containing new node, cost.

    """
    
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = (cur_angle + angle) % 360
    
    new_x = int(np.round(x + L * np.cos(np.radians(new_angle))))
    new_y = int(np.round(y + L * np.sin(np.radians(new_angle))))
    
    move = (new_x, new_y, new_angle)
    cost = L 
    
    return [move,cost]    


def ActionMoveStraight(node, L, angle):
    """
    Performs action corresponding to angle of 0 degrees. 

    Parameters
    ----------
    node : Node
        Object of class Node.
    L : int
        Step-size(stride).
    angle : int
        Angle step-size.

    Returns
    -------
    list
        List containing new node, cost.

    """
    
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = (cur_angle) % 360
    
    new_x = int(np.round(x + L * np.cos(np.radians(new_angle))))
    new_y = int(np.round(y + L * np.sin(np.radians(new_angle))))
    
    move = (new_x, new_y, new_angle)
    cost = L 
    
    return [move,cost]   
 
    
def ActionMoveC30(node, L, angle):
    """
    Performs action corresponding to angle of -30 degrees. 

    Parameters
    ----------
    node : Node
        Object of class Node.
    L : int
        Step-size(stride).
    angle : int
        Angle step-size.

    Returns
    -------
    list
        List containing new node, cost.

    """
    
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = (cur_angle - angle) % 360
    
    new_x = int(np.round(x + L * np.cos(np.radians(new_angle))))
    new_y = int(np.round(y + L * np.sin(np.radians(new_angle))))
    
    move = (new_x, new_y, new_angle)
    cost = L 
    
    return [move,cost]  
 
    
def ActionMoveC60(node, L, angle):
    """
    Performs action corresponding to angle of -60 degrees. 

    Parameters
    ----------
    node : Node
        Object of class Node.
    L : int
        Step-size(stride).
    angle : int
        Angle step-size.

    Returns
    -------
    list
        List containing new node, cost.
    """
    
    x = node.x
    y = node.y
    cur_angle = node.angle
    new_angle = (cur_angle - 2 * angle) % 360
    
    new_x = int(np.round(x + L * np.cos(np.radians(new_angle))))
    new_y = int(np.round(y + L * np.sin(np.radians(new_angle))))
    
    move = (new_x, new_y, new_angle)
    cost = L 
    
    return [move,cost]   


def isGoal(current, goal, radius):
    """
    Checks if a node is in proximity of goal node.

    Parameters
    ----------
    current : Array
        Given node.
    goal : Array
        Goal node.
    radius : int
        Robot radius

    Returns
    -------
    bool
        Whether the node is close enough to be considered for status to be 'goal reached'.

    """
    
    threshold = 1.5 * radius
    distance = Euclidean(current, goal)
    
    if (distance < threshold):
        return True
    else:
        return False

    
def explore(node, c, r, L, angle):
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
    

    moves = [ActionMoveCC60(node, L, angle),
             ActionMoveCC30(node, L, angle),
             ActionMoveStraight(node, L, angle),
             ActionMoveC30(node, L, angle),
             ActionMoveC60(node, L, angle)]
    
    valid_paths = []
    for move in moves:
        if (move[0][0] > 0 and move[0][0] < 400 and move[0][1] > 0 and move[0][1] < 250):
            if not isObstacle(move[0], c, r):
                valid_paths.append([move[0],move[1]])

    return valid_paths


    
def Astar(start_node, goal_node, map, c, r, L, angle):
    """
    Performs A* search.

    Parameters
    ----------
    start_node : Array
        Initial node.
    goal_node : Array
        Goal node.
    map : 2D Array
        Constructed map.
    c : int
        Clearance.
    r : int
        Robot radius.
    L : int
        Step-size(stride).
    angle : int
        Angle step-size.
    

    Returns
    -------
    node_objects : dict
        Dictionary holding information of nodes, instances of class Node.

    """

    print("\n Performing A* search...\n")

    q = PriorityQueue()                                                              # Priority queue for open nodes
    visited = set([])                                                                # Set conataining visited nodes
    node_objects = {}                                                                # dictionary of nodes
    distance = {}                                                                    # distance 
    
    # Assign costs for all nodes to a large value
    for i in range(0, map.shape[1]):
        for j in range(0, map.shape[0]):
            for k in range(0, 360, angle):
                distance[str(([i,j],k))] = 9999999
    
    distance[str(start_node)] = Euclidean(tuple(start_node[0:2]), tuple(goal_node[0:2]))                                                    # Start node has cost of 0
    visited.add(str(start_node))                                                     # Add start node to visited list
    node = Node.Node(start_node,0,None)                                              # Create instance of Node
    node_objects[str(node.pos)] = node                                               # Assigning the node value in dictionary
    q.put((node.cost, node.pos))                                                     # Inserting the start node in priority queue

    counter = 0
    
    while not q.empty(): 
        # if counter == 2:
        #     break                                                            # Iterate until the queue is empty
        node_temp = q.get()                                                          # Pop node from queue
        node = node_objects[str(node_temp[1])]  
        # print(counter)
                                     
        # Check if the node is the goal node
        if isGoal(tuple(node_temp[1][0:2]), tuple(goal_node[0:2]), r):    
            print(" Goal Reached!!!\n")
            print(counter) 
            node_objects[str(goal_node)] = Node.Node(goal_node,node_temp[0], node)
            break
        
        for next_node, cost in explore(node, c, r, L, angle):                                        # Explore neighbors
            if str(next_node) in visited:                                            # Check if action performed next node is already visited
                cost_temp = cost + distance[str(node.pos)] - Euclidean(tuple(node.pos[:2]), tuple(goal_node[:2]))
                if cost_temp < distance[str(next_node)]:                             # Update cost
                    distance[str(next_node)] = cost_temp
                    node_objects[str(next_node)].parent = node
                    
            else:                                                                    # If next node is not visited
                visited.add(str(next_node))
                absolute_cost = cost + distance[str(node.pos)] + Euclidean(tuple(next_node[:2]), tuple(goal_node[:2])) - Euclidean(tuple(node.pos[:2]), tuple(goal_node[:2]))
                distance[str(next_node)] = absolute_cost
                new_node = Node.Node(next_node, absolute_cost, node_objects[str(node.pos)])
                node_objects[str(next_node)] = new_node
                q.put((absolute_cost, new_node.pos))
                        
        counter += 1
    return node_objects


def GeneratePath(node_objects, goal_node):
    """
    Backtracks and finds the path from initial to goal node.

    Parameters
    ----------
    node_objects : dict
        All the explored nodes.
    goal_node : Array
        User defined goal node.

    Returns
    -------
    node_objects : dict
        All the explored nodes.
    path : list
        Path from initial to goal node.

    """
    
    rev_path = []                                                                    # Empty reversed path list 
    goal = node_objects[str(goal_node)]                                              # Get the goal from dictionary
    rev_path.append(goal_node)                                                       # Add the goal to reversed path list 
    parent_node = goal.parent                                                        # Get parent of goal node
    while (parent_node.parent != None):                            
        rev_path.append(parent_node.pos)                                             # Add coordinates of parent node
        parent_node = parent_node.parent                                             # Update parent
    
    path = list(reversed(rev_path))                                                  # Forward path

    return node_objects, path


def Animate(node_objects, path, map):
    """
    Animates the search scenario and exports the animation.

    Parameters
    ----------
    node_objects : dict
        All the explored nodes.
    path : list
        Path from initial to goal node.
    map : 2D Array
        Contructed map.

    Returns
    -------
    None.

    """
    
    print(" Creating animation video...")
    
    width = 400
    height = 250
    FPS = 240                                                                       
    fourcc = VideoWriter_fourcc(*'MP42')
    video = VideoWriter('./Astar.avi', fourcc, float(FPS), (width, height))
    
    
    nodes = node_objects.values()                                                    # Get the values from dictionary(objects of class Node)
    nodes = list(nodes)
    img = np.dstack([map.copy() * 0, map.copy() * 0, map.copy() * 255])              # Convert binary map image to RGB
    img = np.uint8(img)
    video.write(img)

    
    for key in node_objects.keys():
        current_node = node_objects[key]
        if (current_node.parent != None):
            cv2.line(img,(current_node.parent.x,current_node.parent.y),(current_node.x,current_node.y),(100,200,50))
            video.write(img)
               
    for i in range(len(path) - 1):                                                   # Add generated path to video frame 
        if (i==0):
            continue
        cv2.line(img,(path[i-1][0],path[i-1][1]),(path[i][0],path[i][1]),(0,255,255),1)
        video.write(img)

    for i in range(500):
        video.write(img)
            
    
    video.release()
    print(" Animation video saved.")


def initialize():
    """
    Provides solver description and running instructions to the user.

    Returns
    -------
    None.

    """
    
    print("""

                                                                                                                                              
           q   p                                 ,,                             ,,          ,,                          
      db    \ /                        db      `7MM                             db   mm   `7MM                          
     ;MM:o=--*--=o                    ;MM:       MM                                  MM     MM                          
    ,V^MM.  / \                      ,V^MM.      MM  .P"Ybmmm ,pW"Wq.`7Mb,od8 `7MM mmMMmm   MMpMMMb.  `7MMpMMMb.pMMMb.  
   ,M  `MM d   b                    ,M  `MM      MM :MI  I8  6W'   `Wb MM' "'   MM   MM     MM    MM    MM    MM    MM  
   AbmmmqMA            mmmmm        AbmmmqMA     MM  WmmmP"  8M     M8 MM       MM   MM     MM    MM    MM    MM    MM  
  A'     VML                       A'     VML    MM 8M       YA.   ,A9 MM       MM   MM     MM    MM    MM    MM    MM  
.AMA.   .AMMA.                   .AMA.   .AMMA..JMML.YMMMMMb  `Ybmd9'.JMML.   .JMML. `Mbmo.JMML  JMML..JMML  JMML  JMML.
                                                    6'     dP                                                           
                                                    Ybmmmd'                                                             


    --------------------------------------------------------------------------------------------------------------------
         
    THIS PROGRAM USES A* ALGORITHM FOR SEARCHING A PATH FROM USER DEFINED START AND GOAL LOCATION IN A GIVEN MAP FOR A RIGID ROBOT.
    
    --------------------------------------------------------------------------------------------------------------------
    -> The user needs to enter the data for clearance, robot radius, step-size(stride) and angle step-size.
    
    -> Then, the user provides the coordinates of the start and goal node according to the format given below:
    
        For example: For a node with x and y-coordinates as 100 and 200 and orientation as 0, 
            
        Input: 100,200,0
    
    (Note: Only comma seperated values are allowed)
    --------------------------------------------------------------------------------------------------------------------
    """)

