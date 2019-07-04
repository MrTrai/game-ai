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
from moba import *
from astarnavigator import *
from mycreatepathnetwork import *
from mynavigatorhelpers import *
import copy


class MyMinion(Minion):

    def __init__(self, position, orientation, world, image=NPC, speed=SPEED, viewangle=360, hitpoints=HITPOINTS,
                 firerate=FIRERATE, bulletclass=SmallBullet):
        Minion.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass)
        self.states = [Idle, Attack, MovingToTower]

    ### Add your states to self.states (but don't remove Idle)
    ### YOUR CODE GOES BELOW HERE ###

    ### YOUR CODE GOES ABOVE HERE ###

    def start(self):
        Minion.start(self)
        self.changeState(Idle)


############################
### Idle
###
### This is the default state of MyMinion. The main purpose of the Idle state is to figure out what state to change to and do that immediately.

class Idle(State):

    def enter(self, oldstate):
        State.enter(self, oldstate)
        # stop moving
        self.agent.stopMoving()

    def execute(self, delta=0):
        State.execute(self, delta)
        ### YOUR CODE GOES BELOW HERE ###
        self.agent.changeState(MovingToTower)
        ### YOUR CODE GOES ABOVE HERE ###
        return None


##############################
### Taunt
###
### This is a state given as an example of how to pass arbitrary parameters into a State.
### To taunt someome, Agent.changeState(Taunt, enemyagent)

class Taunt(State):

    def parseArgs(self, args):
        self.victim = args[0]

    def execute(self, delta=0):
        if self.victim is not None:
            print "Hey " + str(self.victim) + ", I don't like you!"
        self.agent.changeState(Idle)


##############################
### YOUR STATES GO HERE:
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


class Attack(State):
    def __init__(self):
        self.path = []

    def enter(self, oldstate):
        pass

    def execute(self, delta=0):
        State.execute(self, delta)


def reachTarget(position, dest, nav):
    return distance(position, dest) <= nav.agent.getMaxRadius()


class MovingToTower(State):
    def __init__(self, agent, args=[]):
        super(MovingToTower, self).__init__(agent, args)
        self.path = []
        self.destination = None
        self.curr_pos = None
        self.target = None

    def enter(self, oldstate):
        State.enter(self, oldstate)
        if len(self.path) > 0:
            return None
        nav = self.agent.navigator
        enemy_towers = self.agent.world.getEnemyTowers(self.agent.getTeam())
        self.destination = enemy_towers[0].position
        nav.computePath(self.agent.position, self.destination)
        self.path = copy.copy(nav.path)
        if len(self.path) > 0:
            self.target = self.path[0]
            self.agent.navigator.destination = self.path[0]
            self.path = self.path[1:len(self.path)]

    def execute(self, delta=0):
        State.execute(self, delta)

        nav = self.agent.navigator
        agent_pos = self.agent.position
        world = self.agent.world

        if self.path is None:
            self.path = []
            return

        if self.target is None:
            return

        if len(self.path) == 0:
            network = []
            for edge in nav.pathnetwork:
                if not rayTraceWorld(edge[0], edge[1], world.getLinesWithoutBorders()):
                    network.append(edge)
            network_agent_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, world.getLinesWithoutBorders())
            network_dest_pos = findClosestUnobstructed(self.destination, nav.pathnodes, world.getLinesWithoutBorders())
            new_path, closed_nodes = astar(network_agent_pos, network_dest_pos, network)
            if len(new_path) > 0:
                self.path = new_path
                self.target = self.path[0]
                self.agent.navigator.destination = self.path[0]
                self.path = self.path[1:len(self.path)]

        if not rayTest(agent_pos, self.target, world, self.agent):
            print "Collision Ahead!"
            network = []
            for edge in nav.pathnetwork:
                if not rayTraceWorld(edge[0], edge[1], world.getLinesWithoutBorders()):
                    network.append(edge)
            network_agent_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, world.getLinesWithoutBorders())
            network_dest_pos = findClosestUnobstructed(self.destination, nav.pathnodes, world.getLinesWithoutBorders())
            new_path, closed_nodes = astar(network_agent_pos, network_dest_pos, network)
            if len(new_path) > 0:
                print "Found new path"
                new_path.append(self.destination)
                self.path = new_path
                self.agent.navigator.destination = self.path[0]
                self.target = self.path[0]
                self.path = self.path[1:len(self.path)]

        if reachTarget(agent_pos, self.target, nav) and len(self.path) > 0:
            self.agent.navigator.destination = self.path[0]
            self.target = self.path[0]
            self.path = self.path[1:len(self.path)]

        if self.target:
            self.agent.moveToTarget(self.target)
