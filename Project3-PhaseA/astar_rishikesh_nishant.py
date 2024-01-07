#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENPM 661
Project 3-a

@authors: Rishikesh Jadhav (UID: 119256534) and Nishant Pandey (UID: 119247556)

"""

from functions import *


initialize()

clearance, radius, StepSize, angle = getData()

map_ = create_map(clearance, radius)

start_node = getStartNode(clearance, radius)
goal_node = getGoalNode(clearance, radius)

# nodes = Dijkstra(start_node, goal_node, map)

nodes = Astar(start_node, goal_node, map_, clearance, radius, StepSize, angle)

node_objects, path = GeneratePath(nodes, goal_node)

Animate(node_objects, path, map_)

