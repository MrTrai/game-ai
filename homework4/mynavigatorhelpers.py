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
from Queue import PriorityQueue
from copy import copy


### This function optimizes the given path and returns a new path
### source: the current position of the agent
### dest: the desired destination of the agent
### path: the path previously computed by the A* algorithm
### world: pointer to the world
def is_shortcut_goal(curr_state, destination, path):
    return path[curr_state[-1]] == destination


def next_short_cut_states(curr_state, world, agent, path):
    next_states = []
    start = curr_state[-1]

    for indx in range(start + 1, len(path)):
        if rayTest(path[start], path[indx], world, agent):
            state = copy(curr_state)
            state.append(indx)
            next_states.append(state)

    return next_states


def shortcutPath(source, dest, path, world, agent):
    new_path = [source]
    new_path += path
    new_path += [dest]

    if len(new_path) == 2:
        return new_path

    visit = [
        [0]
    ]

    best_goal = None
    while len(visit) > 0:
        curr_state = visit.pop()

        if is_shortcut_goal(curr_state, dest, new_path):
            if best_goal is None:
                best_goal = curr_state
            elif len(curr_state) < len(best_goal):
                best_goal = curr_state

        for next_state in next_short_cut_states(curr_state, world, agent, new_path):
            visit.append(next_state)

    if best_goal is None:
        return new_path
    result = [new_path[i] for i in best_goal]

    return result


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


def euclideanDist(start, end):
    vect = (abs(start[0] - end[0]), abs(start[1] - end[1]))
    return math.sqrt(
        math.pow(vect[0], 2) + math.pow(vect[1], 2)
    )


### This function changes the move target of the agent if there is an opportunity to walk a shorter path.
### This function should call nav.agent.moveToTarget() if an opportunity exists and may also need to modify nav.path.
### nav: the navigator object
### This function returns True if the moveTarget and/or path is modified and False otherwise
def mySmooth(nav):
    ### YOUR CODE GOES BELOW HERE ###
    agent = nav.agent
    agent_pos = agent.position
    world = nav.world
    resources = world.resources

    if len(world.resources) == 0:
        if not nav.path is None:
            if len(nav.path) > 0:
                agent.stopMoving()
                nav.setPath([])
                nav.agent.targets = []
                nav.agent.target = None
                return True

    min_dist = None
    min_indx = None

    for i, resource in enumerate(resources):
        resrc_pos = resource.position
        if rayTest(agent_pos, resrc_pos, world, agent):
            dist = euclideanDist(agent_pos, resrc_pos)
            if min_dist is None or dist < min_dist:
                min_dist, min_indx = dist, i

    if min_dist:
        agent.moveToTarget(resources[min_indx].position)

    ### YOUR CODE GOES ABOVE HERE ###
    return False
