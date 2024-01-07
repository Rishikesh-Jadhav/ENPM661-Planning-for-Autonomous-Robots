#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: Nishant Pandey and Rishikesh Jadhav
"""

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
import argparse
import numpy as np
from Phase2 import *
import time
import math
def setInitialPos(start):
    print("Trying to set it to initial position")
    rospy.sleep(5.0)
    x = start[0] 
    y = start[1] 
    a = start[2]*math.pi/180.0
    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = np.sin(a/2)  
    state_msg.pose.orientation.w = np.cos(a/2)
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except(rospy.ServiceException):
        print("Service call failed")
    print("Done....Settled to initial position")
    return 

def get_velocities(args, solver):
    r = float(args.WheelRadius)
    l = float(args.WheelLength)

    
    rospy.loginfo(len(solver.trackIndex))

    vx = []
    rz = []
    UL = []
    UR = []
    
    for idx in solver.trackIndex:
        ul, ur = solver.actions[int(idx)] 
        UL.append(ul)
        UR.append(ur)
        vx_ = r*0.5*(ur + ul)
        rz_ = r*(ur - ul)/l
        vx.append(vx_)
        rz.append(rz_)
    
    return UL, UR, vx, rz
        
        
        
def main():
    rospy.init_node('set_pose')
    Parser = argparse.ArgumentParser()
    Parser.add_argument("--Start", default="[0,0,30]", help="Initial location")
    Parser.add_argument('--End', default="[5,0,30]", help='Goal location')
    Parser.add_argument('--RobotRadius', default=0.178, help='Robot radius')
    Parser.add_argument('--Clearance', default=0.05, help='Clearance')
    Parser.add_argument('--ShowExploration', default=1, help='1 for exploration animation else 0')
    Parser.add_argument('--ShowPath', default=1, help='1 to show explored path else 0')
    Parser.add_argument('--thetaStep', default=30, help='Possibilities of action for angle')
    Parser.add_argument('--StepSize', default=2, help='Step size')
    Parser.add_argument('--Threshold', default=0.01, help='Threshold value for approximation')
    Parser.add_argument('--GoalThreshold', default=0.2, help='Threshold for goal')
    Parser.add_argument('--WheelRadius', default=0.033, help='Radius of the robot wheel in meters')
    Parser.add_argument('--WheelLength', default=0.320, help='wheelbase')
    Parser.add_argument('--RPM', default="[15,18]", help='RPM values')
    Parser.add_argument('--Weight', default=1.3, help='Weight for cost to go')

    Args = Parser.parse_args(rospy.myargv()[1:])
    initial = [float(i) for i in Args.Start[1:-1].split(',')]
    end = Args.End
    goal = [float(i) for i in end[1:-1].split(',')] 
    print(goal)
    print(initial)
    r = float(Args.RobotRadius)
    c = float(Args.Clearance)
    StepSize = int(Args.StepSize)
    Threshold = float(Args.Threshold)
    GoalThreshold = float(Args.GoalThreshold)
    wheelLength = float(Args.WheelLength) 
    rpm = [float(i) for i in Args.RPM[1:-1].split(',')]
    Ur, Ul = rpm[0], rpm[1]
    print(Ur, Ul) 
    wheelRadius = float(Args.WheelRadius)
    weight = float(Args.Weight)

    rospy.loginfo("Setting initial position")
    setInitialPos(initial)
    rospy.loginfo("Settled to initial position")

    solver = pathFinder(initial, goal, stepSize=StepSize,
        goalThreshold = GoalThreshold, width = 6, height = 2, threshold = Threshold,
        r=r, c=c, wheelLength = wheelLength, Ur = Ur, Ul = Ul, wheelRadius = wheelRadius,
        weight = weight, showExploration=int(Args.ShowExploration), showPath=int(Args.ShowPath))
    rospy.loginfo("Trying to find the path  "+"--"*50)
    solver.findPath()
    rospy.loginfo("Done "+"="*50)
    
    
    ul, ur, vx, rz = get_velocities(Args, solver)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.angular.z=0
    pub.publish(cmd)
    msg=Twist()
    rate = rospy.Rate(10)
    for i in range(len(vx)):
        rospy.loginfo("ul:{}, ur:{}, vx:{}, rz:{}".format(ul[i],ur[i], vx[i],cmd.angular.z))
        start = time.time()
        counter=0      
        while(time.time() - start <= 1.0):
            # rospy.loginfo(counter)
            cmd.linear.x = vx[i]
            cmd.angular.z = rz[i] 
            pub.publish(cmd)
            rate.sleep()
            counter += 1
            
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
