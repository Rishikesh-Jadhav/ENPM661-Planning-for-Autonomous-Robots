#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENPM 661
Project 2

GITHUB LINK TO THE PROJECT - https://github.com/Rishikesh-Jadhav/ENPM-662-proj2_Rishikesh_Jadhav/tree/master/proj2_Rishikesh_jadhav


@author: Rishikesh Jadhav
UID: 119256534
"""

from functions import *

map = create_map()

initialize()

start_node = getStartNode(map)
goal_node = getGoalNode(map)

nodes = Dijkstra_Algorithm(start_node, goal_node, map)

node_objects, path = GeneratePath(nodes, goal_node)

Animate(node_objects, path, map)

