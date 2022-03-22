#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 21 21:27:54 2022

@author: pulkit
"""

from functions import *


initialize()

clearance, radius, StepSize, angle = getData()

map_ = create_map(clearance, radius)

start_node = getStartNode(clearance, radius)
goal_node = getGoalNode(clearance, radius)

nodes = Astar(start_node, goal_node, map_, clearance, radius, StepSize, angle)
node_objects, path = GeneratePath(nodes, goal_node)

Animate(node_objects, path, map_)