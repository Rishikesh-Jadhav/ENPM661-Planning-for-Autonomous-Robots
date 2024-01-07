#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENPM 661
Project 3-a

@authors: Rishikesh Jadhav (UID: 119256534) and Nishant Pandey (UID: 119247556)

"""

class Node:
    def __init__(self, pos, cost, parent):
        self.pos = pos                      # Position of the node
        self.x = pos[0]                     # x-coordinate of the node
        self.y = pos[1]                     # y-coordinate of the node
        self.cost = cost                    # Total cost to reach the node 
        self.parent = parent                # Parent of the node
        self.angle = pos[2]                 # Orientation

