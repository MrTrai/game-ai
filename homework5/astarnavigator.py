'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy
from pygame.locals import *

from constants import *
from utils import *
from core import *
from mycreatepathnetwork import *
from mynavigatorhelpers import *
from Queue import PriorityQueue

import copy


###############################
### AStarNavigator
###
### Creates a path node network and implements the A* algorithm to create a path to the given destination.

class AStarNavigator(NavMeshNavigator):

    def __init__(self):
        NavMeshNavigator.__init__(self)

    ### Create the path node network.
    ### self: the navigator object
    ### world: the world object
    def createPathNetwork(self, world):
        self.pathnodes, self.pathnetwork, self.navmesh = myCreatePathNetwork(world, self.agent)
        return None

    ### Finds the shortest path from the source to the destination using A*.
    ### self: the navigator object
    ### source: the place the agent is starting from (i.e., its current location)
    ### dest: the place the agent is told to go to
    def computePath(self, source, dest):
        self.setPath(None)
        ### Make sure the next and dist matrices exist
        if self.agent != None and self.world != None:
            self.source = source
            self.destination = dest
            ### Step 1: If the agent has a clear path from the source to dest, then go straight there.
            ###   Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
            ###   Tell the agent to move to dest
            ### Step 2: If there is an obstacle, create the path that will move around the obstacles.
            ###   Find the path nodes closest to source and destination.
            ###   Create the path by traversing the self.next matrix until the path node closest to the destination is reached
            ###   Store the path by calling self.setPath()
            ###   Tell the agent to move to the first node in the path (and pop the first node off the path)
            if clearShot(source, dest, self.world.getLinesWithoutBorders(), self.world.getPoints(), self.agent):
                self.agent.moveToTarget(dest)
            else:
                start = findClosestUnobstructed(source, self.pathnodes, self.world.getLinesWithoutBorders())
                end = findClosestUnobstructed(dest, self.pathnodes, self.world.getLinesWithoutBorders())
                if start != None and end != None:
                    # print len(self.pathnetwork)
                    newnetwork = unobstructedNetwork(self.pathnetwork, self.world.getGates())
                    # print len(newnetwork)
                    closedlist = []
                    path, closedlist = astar(start, end, newnetwork)
                    if path is not None and len(path) > 0:
                        path = shortcutPath(source, dest, path, self.world, self.agent)
                        self.setPath(path)
                        if self.path is not None and len(self.path) > 0:
                            first = self.path.pop(0)
                            self.agent.moveToTarget(first)
        return None

    ### Called when the agent gets to a node in the path.
    ### self: the navigator object
    def checkpoint(self):
        myCheckpoint(self)
        return None

    ### This function gets called by the agent to figure out if some shortcuts can be taken when traversing the path.
    ### This function should update the path and return True if the path was updated.
    def smooth(self):
        return mySmooth(self)

    def update(self, delta):
        myUpdate(self, delta)


def unobstructedNetwork(network, worldLines):
    newnetwork = []
    for l in network:
        hit = rayTraceWorld(l[0], l[1], worldLines)
        if hit == None:
            newnetwork.append(l)
    return newnetwork


def nextStates(curr_state, graph):
    try:
        states = graph[curr_state]
    except Exception:
        states = []
    return states


def euclideanDist(start, end):
    vect = (abs(start[0] - end[0]), abs(start[1] - end[1]))
    return math.sqrt(
        math.pow(vect[0], 2) + math.pow(vect[1], 2)
    )


def isGoal(curr_node, goal):
    return curr_node == goal


def find_dup(arr, destination):
    for i, val in enumerate(arr):
        cost, node, path = val
        if node == destination:
            return i
    return None


def heuristic(curr_node, goal):
    return euclideanDist(curr_node, goal)


def astar(init, goal, network):
    path = []
    open = PriorityQueue()
    closed = []
    graph = {}

    start = (0, init, [init])  # cost, node, path
    open.put(start)

    ### YOUR CODE GOES BELOW HERE ###
    for edge in network:  # build graph from network
        start = edge[0]
        end = edge[1]
        edge_cost = euclideanDist(start, end)
        if start not in graph:
            graph[start] = []
        if end not in graph:
            graph[end] = []

        if end not in graph[start]:
            graph[start].append(
                (end, edge_cost)
            )
        if start not in graph[end]:
            graph[end].append(
                (start, edge_cost)
            )

    while open.qsize() > 0:  # A star Search
        curr_state = open.get()
        curr_cost, curr_node, curr_path = curr_state

        if isGoal(curr_node, goal):
            path = curr_path
            break

        for destination, cost in nextStates(curr_node, graph):
            if destination and goal:
                new_path = copy.copy(curr_path)
                new_path.append(destination)
                new_cost = curr_cost + cost + heuristic(destination, goal)
                new_state = (new_cost, destination, new_path)

                closed_dup = find_dup(closed, destination)
                open_dup = find_dup(open.queue, destination)

                if closed_dup:
                    closed_cost, closed_node, closed_path = closed[closed_dup]
                    if new_cost < closed_cost:
                        open.put(
                            new_state
                        )
                        closed.remove(closed[closed_dup])
                elif open_dup:
                    open_cost, open_node, open_path = open.queue[open_dup]
                    if new_cost < open_cost:
                        open.queue[open_dup] = new_state
                else:
                    open.put(new_state)

        closed.append(curr_state)

    return path, set([node for cost, node, path in closed])


def rayTest(source, dest, world, agent):
    valid = True
    for line in world.getLinesWithoutBorders():
        point1 = line[0]
        point2 = line[1]
        dist1 = minimumDistance((source, dest), point1)
        dist2 = minimumDistance((source, dest), point2)
        intersect = rayTrace(source, dest, line)
        if intersect or dist1 <= agent.getMaxRadius() or dist2 <= agent.getMaxRadius():
            valid = False
    return valid


def myUpdate(nav, delta):
    ### YOUR CODE GOES BELOW HERE ###
    agent_pos = nav.agent.position
    world = nav.world
    agent = nav.agent
    dest = nav.agent.moveTarget
    if not rayTest(agent_pos, dest, world, agent):
        network = nav.pathnetwork
        network_curr_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, nav.world.getLinesWithoutBorders())
        network_dest_pos = findClosestUnobstructed(dest, nav.pathnodes, nav.world.getLinesWithoutBorders())
        new_path, closed_nodes = astar(network_curr_pos, network_dest_pos, network)
        if new_path:
            nav.agent.moveTarget = new_path[0]
            nav.agent.moveToTarget(nav.agent.moveTarget)
            new_path.append(nav.destination)
            nav.setPath(new_path)
        else:
            nav.agent.stopMoving()
    return None


def myCheckpoint(nav):
    ### YOUR CODE GOES ABOVE HERE ###
    agent_pos = nav.agent.position
    world = nav.world
    agent = nav.agent
    dest = nav.agent.moveTarget
    if not rayTest(agent_pos, dest, world, agent):
        network = nav.pathnetwork
        network_curr_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, nav.world.getLinesWithoutBorders())
        network_dest_pos = findClosestUnobstructed(nav.destination, nav.pathnodes, nav.world.getLinesWithoutBorders())
        new_path, closed_nodes = astar(network_curr_pos, network_dest_pos, network)
        if new_path:
            nav.agent.moveTarget = new_path[0]
            nav.agent.moveToTarget(nav.agent.moveTarget)
            new_path.append(nav.destination)
            nav.setPath(new_path)
        else:
            nav.agent.stopMoving()
    ### YOUR CODE GOES ABOVE HERE ###
    return None


### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
    ### YOUR CODE GOES BELOW HERE ###
    direction = (p1, p2)
    valid = True

    for edge in worldLines:
        point1 = edge[0]
        point2 = edge[1]
        intersect = rayTrace(point1, point2, direction)
        if intersect:
            valid = False

    for point in worldPoints:
        min_dist = minimumDistance(direction, point)
        if min_dist <= agent.getMaxRadius():
            valid = False

    return valid
