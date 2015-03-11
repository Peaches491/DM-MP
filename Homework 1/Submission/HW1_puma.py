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

    PumaTransform = array([[1, 0, 0, -3.3232],
                           [0, 1, 0, -0.4878],
                           [0, 0, 1,  0.0000],
                           [0, 0, 0,  1]])

    robot2 = env.ReadRobotXMLFile('robots/pumaarm.zae')
    with env:
        robot2.SetTransform(PumaTransform)
        env.Add(robot2, True)

    CameraTransform = array([[-0.23753303, -0.35654009,  0.90358023, -4.93138313],
                             [-0.97133858,  0.07864789, -0.22431198, -0.44495657],
                             [ 0.00891154, -0.93096384, -0.36500262,  1.32067215],
                             [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]])

    view = env.GetViewer()
    view.SetCamera(CameraTransform)



    set_arm_pose(robot, [0, 0, 0, 0, 0, 0, 0], is_left=True)
    print "Collision Results:", env.CheckCollision(robot, robot2)

    raw_input("Press enter to move to collision...")
    set_arm_pose(robot, [1.374, 1.200, 0, -2.2, 3.14, -1, 0], is_left=True)
    print "Collision Results:", env.CheckCollision(robot, robot2)


    # Code for running input loop, prompting for comma-separated joint values
    if False:
        import StringIO
        previous = "0, 0, 0, 0, 0, 0, 0"
        inp = "1234"
        while 'quit' not in inp:
            inp = raw_input("Testing: ")
            values = [float(x) for x in inp.split(',')]
            if len(values) is 7:
                set_arm_pose(robot, values, is_left=True)
            else:
                print "Need 7 joint values"
            time.sleep(0.01)
    else:
        raw_input("Press enter to exit...")

    # #### END OF YOUR CODE ###
