#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENPM 661
Project 2

@author: Rishikesh Jadhav
UID: 119256534
"""


# IMPORTING PACKAGES

import numpy as np
from queue import PriorityQueue
from cv2 import VideoWriter, VideoWriter_fourcc 
import Node


# FUNCTION DEFINITIONS

def line(p1, p2, x, y, t):
    """
    Constructs a line passing through two points and calculates the distance of a given point from the line. 
    """
    
    d = ((p2[1] - p1[1]) * (x - p1[0])) / ( p2[0] - p1[0]) + (p1[1] + t - y)
    
    return d



def create_map():
    """
    Creates the given map in a numpy array.

    Returns
    -------
    map : 2D Array
        Created map.

    """
    # map = np.zeros((250,400))
    map = np.zeros((250,600))

    
    r = 0                                       # radius
    c = 5                                       # clearance
    t = r + c                                   # Total clearance

    # Clearance is also added in the same color code, so the obstacles are appear to be inflated.
   
    for i in range(map.shape[1]):
        for j in range(map.shape[0]):
            
            # Hexagonal Obastacle
            if (i > (235.05-t) and i < (364.95+t) and line((235.05,87.5),(300,50),i,j,t) < 0 
                and line((300,50),(364.95,87.5),i,j,t) < 0 
                and line((235.05,162.5),(300,200),i,j,t) > 0 
                and line((300,200),(364.95,162.5),i,j,t) > 0):
                map[j,i] = 1
            

            # lower rectangle
            if (i > (100-t) and i < (150+t) and line((100,150),(150,150),i,j,t) < 0 
                and line((100,250),(150,250),i,j,t) > 0):
                map[j,i] = 1
                        


            # upper rectangle
            if (i > (100-t) and i < (150+t) and line((100,0),(150,0),i,j,t) < 0 
                and line((100,100),(150,100),i,j,t) > 0):
                map[j,i] = 1


            if (i>(460-t)and i<(510+t)and j>(25) and j< (225+2*t) and line((460,225),(510,125),i,j,t) > 0
               and line((460,25),(510,125),i,j,t) < 0):
                map[j,i]=1

            # Boundaries condition of the map extreities so that obstacles are not there.
            if (i > 0 and i < t):
                map[j,i] = 1
            if (i < 600 and i > (600-t)):
                map[j,i] = 1
            if (j > 0 and j < t):
                map[j][i] = 1
            if (j < 250 and j >(250 - t)):
                map[j][i] = 1    
    return map


def isObstacle(point,map):
    """
    Checks whether the point collides with an obstacle.

    """
    
    flag = False
    
    if map[point[1],point[0]] == 1:
        flag = True
    
    return flag
    

def getStartNode(map):
    """
    Gets the start node from the user.

    """
    
    flag = False
    while not flag:
        start_node = [int(item) for item in input("\n Please enter the start node: ").split(',')]
        start_node[1] = 250 - start_node[1]
        # check 1 - range of map
        if (len(start_node) == 2 and (0 <= start_node[0] <= 600) and (0 <= start_node[1] <= 250)):
        # check 2 - obstacle collision?
            if not isObstacle(start_node,map):
                flag = True
            else:   
                print("Start node collides with obstacle \n")
        else:
            print("The input node location does not exist in the map, please enter a valid start node.\n")
            flag = False
      
    return start_node


def getGoalNode(map):
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
        
        # check 1 - range of map       
        if (len(goal_node) == 2 and (0 <= goal_node[0] <= 600) and (0 <= goal_node[1] <= 250)):
        # check 2 - obstacle collision?
            if not isObstacle(goal_node,map):
                flag = True
            else:
                print("Goal node collides with obstacle \n")
        else:
            print("The input node location does not exist in the map, please enter a valid goal node.\n")
            flag = False
        
    return goal_node


def explore(node,map):
    """
    Explores the neighbors of the current node and performs move action.

    """
    x = node.x
    y = node.y

    moves = [(x, y + 1),(x + 1, y),(x, y -1),(x - 1, y),(x + 1, y + 1),(x + 1, y - 1),(x - 1, y - 1),(x - 1, y + 1)]
    valid_paths = []
    for pos, move in enumerate(moves):
        if not (move[0] >= 600 or move[0] < 0 or move[1] >= 250 or move[1] < 0):
            if map[move[1]][move[0]] == 0:
                cost = 1.4 if pos > 3 else 1
                valid_paths.append([move,cost])

    return valid_paths


def Dijkstra_Algorithm(start_node, goal_node, map):
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
    
    # Assign costs for all nodes to a large value (infinity)
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

    # """
    rev_path = []                                                                    # Empty reversed path list 
    goal = node_objects[str(goal_node)]                                              # Get the goal from dictionary
    rev_path.append(goal_node)                                                       # Add the goal to reversed path list 
    parent_node = goal.parent                                                        # Get parent of goal node
    while parent_node:                              
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
    
    width = 600
    height = 250
    FPS = 240                                                                       
    fourcc = VideoWriter_fourcc(*'mp4v')
    video = VideoWriter('./Djikstra_algo.mp4', fourcc, float(FPS), (width, height))
    
    
    nodes = node_objects.values()                                                    # Get the values from dictionary(objects of class Node)
    nodes = list(nodes)
    img = np.dstack([map.copy() * 0, map.copy() * 0, map.copy() * 255])              # Convert binary map image to RGB
    img = np.uint8(img)
    video.write(img)
    
    
    for i in range(len(nodes)):                                                      # Add visited nodes to video frame
        img[nodes[i].pos[1], nodes[i].pos[0], :] = np.array([0,255,10])
        video.write(img)
        
    for i in range(len(path) - 1):                                                   # Add generated path to video frame 
        img[path[i][1], path[i][0], :] = np.array([255,10,0])
        video.write(img)
    
    video.release()
    print(" Animation video saved.")
 
    
def initialize():
    """
    Displays information to the user about the solver and how to run it.

    Returns
    -------
    None.

    """

    print("""
    --------------------------------------------------------------------------------------------------------------------
    This program utilizes Dijkstra's algorithm to search for a path between a user-defined start and goal location on a given map.
    --------------------------------------------------------------------------------------------------------------------
    
    -> The user must provide the coordinates of the start and goal node in the following format:
    
        For example: For a node with x and y-coordinates of 100 and 200, 
            
        Input: 100,200
    
    (Note: Only comma-separated values are permitted)
    --------------------------------------------------------------------------------------------------------------------
    """)
