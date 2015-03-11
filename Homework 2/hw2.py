#!/usr/bin/env python
# -*- coding: utf-8 -*-
# HW1 for RBE 595/CS 525 Motion Planning
# code based on the simplemanipulation.py example
from _ast import operator

import time
import operator
import itertools
from math import *
from numpy import *
from Queue import PriorityQueue
from collections import OrderedDict
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


class AStar:
    def __init__(self, _neighbor_test, minimums, maximums, steps, connected_axes=1, axis_identification=None, euclidian=True):
        if len(minimums) is not len(maximums) is not len(steps):
            raise ValueError("A Star requires three arrays of the same length-" +
                             " Minimums: " + len(minimums) +
                             " Maximums: " + len(maximums) +
                             " Steps: " + len(steps))
        if connected_axes > len(minimums):
            raise ValueError("A Star requires connected axes less than or equal to the dimensionality of the space" +
                             " Connected Axes: " + connected_axes +
                             " Minimums: " + len(minimums) +
                             " Maximums: " + len(maximums) +
                             " Steps: " + len(steps))

        self.neighbor_test = _neighbor_test
        self.minimums = minimums
        self.maximums = maximums
        self.steps = steps
        self.grid_size = [(_max - _min) / float(_step)
                          for _min, _max, _step
                          in zip(self.minimums, self.maximums, self.steps)]
        self.axis_identification = axis_identification

        if euclidian:
            self._metric = self._euclidian
        else:
            self._metric = self._manhattan

        self.red_points = zeros([product(self.steps)*1.2, 3])
        self.green_points = zeros([product(self.steps)*1.22, 3])
        self.red_handle = None
        self.red_idx = 0
        self.green_handle = None
        self.green_idx = 0

        if connected_axes is 1:
            self.neighbor_offsets = list(OrderedDict.fromkeys([x for x in itertools.permutations([0, 0, 1])]))
            self.neighbor_offsets.extend(list(OrderedDict.fromkeys([x for x in itertools.permutations([0, 0, -1])])))
            self.neighbor_offsets.sort()
            self.neighbor_offsets = list(OrderedDict.fromkeys(self.neighbor_offsets))
        else:
            _tmp_offsets = list()
            for i in range(connected_axes):
                _tmp_offsets.extend([-1, 1, 0])
            self.neighbor_offsets = [x for x in itertools.permutations(_tmp_offsets, 3)]
            self.neighbor_offsets.sort(cmp=lambda a, b: sum([abs(i) for i in a]) - sum([abs(i) for i in b]))
            self.neighbor_offsets = list(OrderedDict.fromkeys(self.neighbor_offsets))
            if tuple([0 for x in range(connected_axes)]) in self.neighbor_offsets:
                self.neighbor_offsets.remove(tuple([0 for x in range(connected_axes)]))

    def grid_to_pos(self, cell):
        return tuple([_val * _size + _min
                      for _val, _size, _min
                      in zip(cell, self.grid_size, self.minimums)])

    def pos_to_grid(self, pos):
        return tuple([round((_val - _min) * _steps / (_max - _min - .0))
                     for _val, _steps, _max, _min
                     in zip(pos, self.steps, self.maximums, self.minimums)])

    def limit_check(self, n):
        _pos = self.grid_to_pos(n)
        for _max, _min, _val in zip(self.maximums, self.minimums, _pos):
            if _min > _val or _max < _val:
                return False
        return True

    @staticmethod
    def _vec_sum(a, b):
        return tuple(map(operator.add, a, b))

    def get_neighbors(self, current_cell):
        neighbors = [self._vec_sum(current_cell, off) for off in self.neighbor_offsets]
        if False:
            return [n for n in neighbors if self.limit_check(n) and self.neighbor_test(self.grid_to_pos(n))]
        else:
            filtered_neighbors = list()
            for n in neighbors:
                if self.limit_check(n) and self.neighbor_test(self.grid_to_pos(n)):
                    filtered_neighbors.append(n)
                else:
                    rad = 0.4 * min(self.grid_size[:-1])
                    _pos = self.grid_to_pos(n)
                    self.red_points[self.red_idx] = [_pos[0] + cos(_pos[2]) * rad,
                                                     _pos[1] + sin(_pos[2]) * rad,
                                                     0.028]
                    self.red_idx += 1
                    if self.red_idx % 100 is 0:
                        self.red_handle = env.plot3(points=self.red_points,
                                                    pointsize=2.0,
                                                    colors=array([1, 0, 0]))
            return filtered_neighbors

    def cost(self, a, b):
        return self._metric(self.grid_to_pos(a), self.grid_to_pos(b))

    def heuristic(self, a, b):
        return self._metric(self.grid_to_pos(a), self.grid_to_pos(b))

    @staticmethod
    def _manhattan(a, b):
        return linalg.norm([abs(b - a) for a, b in zip(a, b)], ord=1)

    @staticmethod
    def _euclidian(a, b):
        return linalg.norm([abs(b - a) for a, b in zip(a, b)], ord=2)

    def a_star_search(self, start_pos, goal_pos):

        start = self.pos_to_grid(start_pos)
        goal = self.pos_to_grid(goal_pos)

        print start_pos, "->", start
        print goal_pos, "->", goal

        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0

        ctr = 0
        while not frontier.empty():
            pri, current = frontier.get()

            ctr += 1
            rad = 0.4 * min(self.grid_size[:-1])
            _pos = self.grid_to_pos(current)

            self.green_points[self.green_idx] = [_pos[0] + cos(_pos[2]) * rad,
                                                 _pos[1] + sin(_pos[2]) * rad,
                                                 0.025]
            self.green_idx += 1
            if self.green_idx % 100 is 0:
                self.green_handle = (env.plot3(points=self.green_points,
                                               pointsize=2.0,
                                               colors=array([0, 0, 1])))

            if current == goal:
                print "Path found!"
                path = [goal]
                cell = goal
                while cell in came_from.keys():
                    cell = came_from[cell]
                    path.append(cell)
                path.reverse()
                return map(self.grid_to_pos, path[1:])

            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.cost(self.grid_to_pos(current), self.grid_to_pos(next))
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(self.grid_to_pos(next), self.grid_to_pos(goal))
                    frontier.put((priority, next))
                    came_from[next] = current

        print "No path found!"
        return []


def wait_robot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


def tuck_arms(env, robot):
    with env:
        joint_names = ['l_shoulder_lift_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint',
                       'r_shoulder_lift_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in joint_names])
        robot.SetActiveDOFValues([1.29023451, -2.32099996, -0.69800004,
                                  1.27843491, -2.32100002, -0.69799996])
        robot.GetController().SetDesired(robot.GetDOFValues())
    wait_robot(robot)


def test_pose(env, x, y, theta, z=0.005):
    xform = array([[cos(theta), -sin(theta), 0, x],
                   [sin(theta),  cos(theta), 0, y],
                   [0,           0,          1, z],
                   [0,           0,          0, 1]])
    with env:
        robot.SetTransform(xform)
        result = env.CheckCollision(robot)
    return not result

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
    tuck_arms(env, robot)
    wait_robot(robot)
    time.sleep(1)

    with env:

        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])

        start_config = (-3.4, -1.4, 0)
        goal_config = (2.6, -1.3, -pi / 2)

        robot.SetActiveDOFValues(start_config)

        # ### YOUR CODE HERE ####
        handles = list()
        handles.append(env.plot3(points=array([goal_config[0], goal_config[1], 1]),
                                 pointsize=10.0,
                                 colors=array([0, 1, 0])))

        camera_dist = 8
        CameraTransform = eye(4)
        CameraTransform[1, 1] = -1
        CameraTransform[2, 2] = -1
        CameraTransform[2, 3] = camera_dist
        view = env.GetViewer()
        view.SetCamera(CameraTransform, camera_dist)

        # ### Implement the A* algorithm to compute a path for the robot's base starting from the
        # current configuration of the robot and ending at goal_config. The robot's base DOF have
        # already been set as active. It may be easier to implement this as a function in a
        # separate file and call it here.

        aabb = env.GetKinBody('ProjectRoom').ComputeAABB()
        pos = aabb.pos()
        extents = aabb.extents()
        print pos, extents

        scaling = 0.5

        x_min = -1 * extents[0] + pos[0]
        x_max = extents[0] + pos[0]
        x_count = 128 * scaling

        y_min = -1 * extents[1] + pos[1]
        y_max = extents[1] + pos[1]
        y_count = 64 * scaling

        theta_min = -1 * pi
        theta_max = pi
        theta_count = 32 * scaling

        def collision_test(_pos):
            return test_pose(env, *_pos)

        star = AStar(collision_test, [x_min, y_min, theta_min],
                     [x_max, y_max, theta_max],
                     [x_count, y_count, theta_count],
                     connected_axes=3, euclidian=True)

        print "[0, 0, 0], [1, 0, 0] ->", star.cost([0, 0, 0], [1, 0, 0])
        print "[0, 0, 0], [1, 1, 0] ->", star.cost([0, 0, 0], [1, 1, 0])
        print "[0, 0, 0], [1, 1, 0] ->", star.cost([0, 0, 0], [1, 1, 1])

        print "[0, 0, 0], [-1,  0,  0] ->", star.cost([0, 0, 0], [-1,  0,  0])
        print "[0, 0, 0], [-1, -1,  0] ->", star.cost([0, 0, 0], [-1, -1,  0])
        print "[0, 0, 0], [-1, -1, -1] ->", star.cost([0, 0, 0], [-1, -1, -1])

        path = list()
        path = star.a_star_search(start_config, goal_config)

        # ### Draw your path in the openrave here
        if not len(path) is 0:
            np_path = array(path)
            np_path[:, 2] = 0.02
            print np_path
            print np_path.shape

            col = (0, 0, 0)
            handles.append(env.drawlinestrip(points=np_path,
                                             linewidth=3.0,
                                             colors=array([col for step in path])))

        # ### Draw the X and Y components of the configurations explored by A*

        # ### Now that you have computed a path, execute it on the robot using the controller. You
        # will need to convert it into an openrave trajectory. You can set any reasonable timing
        # for the configurations in the path. Then, execute the trajectory using
        # robot.GetController().SetPath(mypath);

        theta = -pi / 2
        xform = array([[cos(theta), -sin(theta), 0, start_config[0]],
                       [sin(theta),  cos(theta), 0, start_config[1]],
                       [0,           0,          1, 0.05],
                       [0,           0,          0, 1]])
        robot.SetTransform(xform)
        robot.GetController().SetDesired(robot.GetDOFValues())
    wait_robot(robot)

    while True:
        with env:
            robot.SetActiveDOFValues(start_config)
            traj = RaveCreateTrajectory(env, '')
            traj.Init(robot.GetActiveConfigurationSpecification())
            traj.Insert(0, robot.GetActiveDOFValues())

            i = 0
            for i, point in enumerate(path[:-1]):
                traj.Insert(i + 1, point)
            traj.Insert(i + 2, goal_config)

            planningutils.RetimeActiveDOFTrajectory(traj, robot, hastimestamps=False, maxvelmult=3, maxaccelmult=1)
            print 'duration', traj.GetDuration()

            robot.GetController().SetPath(traj)
            # robot.WaitForController(0)

            # ### END OF YOUR CODE ###

        wait_robot(robot)
        raw_input("Press enter to exit...")