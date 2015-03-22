#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import multiprocessing, math, time
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint',
                      'r_shoulder_lift_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451, -2.32099996, -0.69800004, 1.27843491, -2.32100002, -0.69799996])
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('rrtplugin/build/rrtplugin')
    MyNewModule = RaveCreateModule(env, 'RRTModule')
    ### END INITIALIZING YOUR PLUGIN ###


    # tuck in the PR2's arms for driving
    tuckarms(env, robot)

    #set active joints
    jointnames = ['l_shoulder_pan_joint',   'l_shoulder_lift_joint', 'l_elbow_flex_joint',
                  'l_upper_arm_roll_joint', 'l_forearm_roll_joint',  'l_wrist_flex_joint',
                  'l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

    # set start config
    # startconfig = [0.449, -0.201, -0.8, 0, 0, -0.8, 0]
    startconfig = [-0.15, 0.075, -1.008, 0, 0, 0, 0]
    robot.SetActiveDOFValues(startconfig)
    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    tmp = False

    with env:
    # for i in range(1):
        jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

        goalconfig = [0.449, -0.201, 0, 0, 0, 0, 0]

        # ### YOUR CODE HERE ###
        # Call your plugin to plan, draw, and execute a path
        # from the current configuration of the left arm to
        # the goalconfig


        cmd = 'RunRRT '
        cmd += '-n ' + str(len(goalconfig)) + ' '
        # cmd += '-s ' + str([float(x) for x in robot.GetActiveDOFValues()]).translate(None, "[],") + ' '
        cmd += '-s ' + str([float(x) for x in startconfig]).translate(None, "[],") + ' '
        cmd += '-g ' + str(goalconfig).translate(None, "[],") + ' '
        cmd += '-d ' + str(0.05) + ' '
        cmd += '-f ' + str(0.05) + ' '
        # cmd += '-f ' + str(1.00) + ' '
        cmd += '-i ' + str([int(robot.GetJointFromDOFIndex(x).IsCircular(0)) for x in robot.GetActiveDOFIndices()]).translate(None, "[],") + ' '

        print "Sending command"
        print cmd


        if False:
            dist = math.sqrt(math.fsum([(a-b)**2 for a, b in zip(goalconfig, startconfig)]))
            steps = math.floor(math.sqrt(math.fsum([(a-b)**2 for a, b in zip(goalconfig, startconfig)])) / 0.05)
            increments = [(a-b)/steps for a, b in zip(goalconfig, startconfig)]

            current_step = startconfig
            for i in range(0, int(steps+1)):
                current_step = [a+(b*i) for a, b in zip(startconfig, increments)]
                print i, current_step

                with env:
                    robot.SetActiveDOFValues(current_step)
                    robot.GetController().SetDesired(robot.GetDOFValues())
                waitrobot(robot)
                print "Collides:", env.CheckCollision(robot)
                time.sleep(0.5)
        else:
            print MyNewModule.SendCommand(cmd)

        # ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")


