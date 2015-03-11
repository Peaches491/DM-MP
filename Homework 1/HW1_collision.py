#!/usr/bin/env python
# -*- coding: utf-8 -*-
# HW1 for RBE 595/CS 525 Motion Planning
# code based on the simplemanipulation.py example

import time
import sys
import openravepy

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


def set_arm_pose(robot, joint_values, is_left = True):
    joint_set = []
    if is_left:
        joint_set = ['l' + j for j in _arm_joint_names]
    else:
        joint_set = ['r' + j for j in _arm_joint_names]

    with env:
        robot.SetActiveDOFs([robot.GetJoint(j_name).GetDOFIndex() for j_name in joint_set])
        robot.SetActiveDOFValues(joint_values)
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

    PumaTransform = array([[1, 0, 0, -0.07535],
                           [0, 1, 0, -0.02555],
                           [0, 0, 1,  0.0100],
                           [0, 0, 0,  1]])

    # Place the robot at near the wall, waiting for the changes to take hold.
    with env:
        robot.SetTransform(PumaTransform)
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    CameraTransform = openravepy_int.matrixFromAxisAngle([-0.220173, 0.828423, -0.515013], (168.85*pi)/180)
    CameraTransform[0, 3] = -1.222979
    CameraTransform[1, 3] = 3.284817
    CameraTransform[2, 3] = 2.728495

    view = env.GetViewer()
    view.SetCamera(CameraTransform)

    print "Collision Before arm move:", env.CheckCollision(robot)
    raw_input("Press enter to move to collision...")


    joint_set = []
    joint_set = ['l' + j for j in _arm_joint_names]

    with env:
        robot.SetActiveDOFs([robot.GetJoint(j_name).GetDOFIndex() for j_name in joint_set])
        robot.SetActiveDOFValues([0, 0, 0, 0, 0, 0, 0])
        print "Collision After arm move:", env.CheckCollision(robot)
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)
    print "Collision After wait:", env.CheckCollision(robot)

    raw_input("Press enter to exit...")

    # #### END OF YOUR CODE ###
