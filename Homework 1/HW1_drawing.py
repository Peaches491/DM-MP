#!/usr/bin/env python
# -*- coding: utf-8 -*-
# HW1 for RBE 595/CS 525 Motion Planning
# code based on the simplemanipulation.py example

import time
from math import *
import openravepy
import numpy as np
from openravepy import *
from numpy import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


_arm_joint_names = \
    ['_shoulder_pan_joint',
     '_shoulder_lift_joint',
     '_upper_arm_roll_joint',
     '_elbow_flex_joint',
     '_forearm_roll_joint',
     '_wrist_flex_joint',
     '_wrist_roll_joint']


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


def tuckarms(env, robot):
    with env:
        jointnames = ['l_shoulder_lift_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint',
                      'r_shoulder_lift_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451, -2.32099996, -0.69800004,
                                  1.27843491, -2.32100002, -0.69799996])
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside   the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env, robot)

    # #### YOUR CODE HERE ####

    # First, move the camera to a good location to view all drawing elements

    # CameraTransform = array([[ 0.91343,  0.13654, -0.38275,   4.596312],
    #                          [ 0.40635, -0.31740,  0.85671, -10.257668],
    #                          [-0.00445, -0.93831, -0.34550,   4.201685],
    #                          [ 0.00000,  0.00000,  0.00000,   1.000000]])

    camera_dist = 13
    CameraTransform = eye(4)
    CameraTransform[1, 1] = -1
    CameraTransform[2, 2] = -1
    CameraTransform[2, 3] = camera_dist
    view = env.GetViewer()
    view.SetCamera(CameraTransform, camera_dist)

    handles = []

    # #### Draw red rectangles around every table in the environment
    #      First, get the names of each body with the word "Table" in it
    tables = [b.GetName() for b in env.GetBodies() if "Table" in b.GetName()]

    #      Then, for each table, generate a line strip, and append it.
    #      The Axis-Aligned Bounding Box for each table defines the following:
    #        pos - The postion of the centroid of the bounding box
    #        extents - Half the length, width, and height of the ABB
    for table_name in tables:
        aabb = env.GetKinBody(table_name).ComputeAABB()
        pos = aabb.pos()
        extents = aabb.extents()
        col = (1, 0, 0)
        pts = array( ( pos[:]+extents*[-1, -1,  1],
                       pos[:]+extents*[ 1, -1,  1],
                       pos[:]+extents*[ 1,  1,  1],
                       pos[:]+extents*[-1,  1,  1],
                       pos[:]+extents*[-1, -1,  1],) )
        handles.append(env.drawlinestrip(points=pts,
                                         linewidth=3.0,
                                         colors=array((col, col, col, col, col,))))

    # #### Draw Circle of points around scene
    circle_points = []
    circle_colors = []
    rad = 6
    #      Generate the circle of points, and their colors
    for i in linspace(0, 2*pi, 64, endpoint=False):
        circle_points.append((sin(i)*rad, cos(i)*rad, 0))
        circle_colors.append((0, 0, 1))
    #      Add circle points to environment
    handles.append(env.plot3(points=array(circle_points),
                             pointsize=10.0,
                             colors=array(circle_colors)))

    # #### END OF YOUR CODE ###

    raw_input("Press enter to exit...")