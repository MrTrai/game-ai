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
        self.states = [Idle, MoveToEnemyBase, AttackMinion, AttackTower, AttackBase, Strafe]

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
        self.agent.changeState(MoveToEnemyBase)
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


def reachTarget(position, dest, nav):
    return distance(position, dest) <= nav.agent.getMaxRadius()


def isEnemyNPCAround(moba_agents, agent_pos, team, radius=BULLETRANGE):
    for moba_agent in moba_agents:
        dist = euclideanDist(moba_agent.position, agent_pos)
        if moba_agent.getTeam() != 0 and moba_agent.getTeam() != team and dist <= radius:
            return True
    return False


def getEnemies(moba_agents, agent_pos, team, radius=BULLETRANGE):
    enemies = []
    for moba_agent in moba_agents:
        dist = euclideanDist(moba_agent.position, agent_pos)
        if moba_agent.getTeam() != 0 and moba_agent.getTeam() != team and dist < radius:
            enemies.append(moba_agent)
    return enemies


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


def getEnemyTowers(enemy_towers, agent_position, BULLETRANGE):
    towers = []
    for tower in enemy_towers:
        dist = euclideanDist(tower.position, agent_position)
        if dist <= BULLETRANGE:
            towers.append(tower)
    return towers


def isEnemyTowerAround(enemy_towers, agent, BULLETRANGE):
    for tower in enemy_towers:
        dist = euclideanDist(tower.position, agent.position)
        if dist <= BULLETRANGE:
            return True
    return False


def isEnemyBaseAround(enemy_bases, agent, BULLETRANGE):
    for base in enemy_bases:
        dist = euclideanDist(base.position, agent.position)
        if dist <= BULLETRANGE:
            return True
    return False


class AttackTower(State):
    def __init__(self, agent, args=[]):
        super(AttackTower, self).__init__(agent, args)
        self.enemy_towers = []

    def enter(self, oldstate):
        # print "Attack Tower"
        pass

    def execute(self, delta=0):
        State.execute(self, delta)
        agent_team = self.agent.getTeam()
        self.enemy_towers = getEnemyTowers(self.agent.world.getEnemyTowers(agent_team), self.agent.position,
                                           BULLETRANGE)
        if len(self.enemy_towers) > 0:
            self.agent.stopMoving()
            self.agent.turnToFace(self.enemy_towers[0].position)
            self.agent.shoot()
        else:
            self.agent.changeState(MoveToEnemyBase)

    def parseArgs(self, args):
        pass


def getNearBullets(bullets, agent):
    near_bullets = []
    agent_pos = agent.position
    for bullet in bullets:
        dist = euclideanDist(bullet.position, agent_pos)
        if dist <= BULLETRANGE:
            near_bullets.append(bullet)
    return near_bullets


class Strafe(State):
    def __init__(self, agent, args=[]):
        super(Strafe, self).__init__(agent, args)
        self.last_state = None
        self.destination = None

    def enter(self, oldstate):
        # print "Strafe"
        pass

    def execute(self, delta=0):
        State.execute(self, delta)
        near_bullets = getNearBullets(self.agent.world.getBullets(), self.agent)
        for near_bullet in near_bullets:
            if not self.destination:
                pass


    def parseArgs(self, args):
        self.last_state = args[0]


class AttackBase(State):
    def __init__(self, agent, args=[]):
        super(AttackBase, self).__init__(agent, args)
        self.enemy_bases = []

    def enter(self, oldstate):
        # print "Attack Base"
        pass

    def execute(self, delta=0):
        State.execute(self, delta)
        agent_team = self.agent.getTeam()
        self.enemy_bases = self.agent.world.getEnemyBases(agent_team)
        if len(self.enemy_bases) > 0:
            self.agent.stopMoving()
            self.agent.turnToFace(self.enemy_bases[0].position)
            self.agent.shoot()
        else:
            self.agent.changeState(MoveToEnemyBase)

    def parseArgs(self, args):
        pass


def isBulletNear(bullets, agent):
    agent_pos = agent.position
    for bullet in bullets:
        dist = euclideanDist(bullet.position, agent_pos)
        if dist <= BULLETRANGE:
            return True
    return False


class AttackMinion(State):
    def __init__(self, agent, args=[]):
        super(AttackMinion, self).__init__(agent, args)
        self.enemy_minions = []

    def enter(self, oldstate):
        # print "Attack Minion"
        pass

    def execute(self, delta=0):
        State.execute(self, delta)
        bullets = self.agent.world.getBullets()

        if isBulletNear(bullets=bullets, agent=self.agent):
            self.agent.changeState(Strafe, AttackMinion)
            return

        visibles = self.agent.getVisibleType(MOBAAgent)
        agent_team = self.agent.getTeam()
        self.enemy_minions = getEnemies(visibles, self.agent.position, agent_team, BULLETRANGE)
        if len(self.enemy_minions) > 0:
            self.agent.stopMoving()
            self.agent.turnToFace(self.enemy_minions[0].position)
            self.agent.shoot()
        else:
            self.agent.changeState(MoveToEnemyBase)

    def parseArgs(self, args):
        pass


class MoveToEnemyBase(State):
    def __init__(self, agent, args=[]):
        super(MoveToEnemyBase, self).__init__(agent, args)
        self.path = []
        self.destination = None
        self.curr_pos = None
        self.target = None

    def enter(self, oldstate):
        # print "Move"
        State.enter(self, oldstate)
        enemy_base = self.agent.world.getEnemyBases(self.agent.getTeam())
        nav = self.agent.navigator
        agent_pos = self.agent.position
        world = self.agent.world

        if len(enemy_base) > 0:
            self.destination = enemy_base[0].position
            network_agent_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, world.getLinesWithoutBorders())
            network_dest_pos = findClosestUnobstructed(self.destination, nav.pathnodes, world.getLinesWithoutBorders())
            nav.computePath(network_agent_pos, network_dest_pos)
            self.path = copy.copy(nav.path)

        if not self.path:
            self.path = []

        if len(self.path) > 0:
            self.target = self.path[0]
            self.agent.navigator.destination = self.path[0]
            self.path = self.path[1:len(self.path)]

    def execute(self, delta=0):
        State.execute(self, delta)

        if not self.destination:
            return

        agent_team = self.agent.getTeam()
        nav = self.agent.navigator
        agent_pos = self.agent.position
        world = self.agent.world

        if isEnemyTowerAround(self.agent.world.getEnemyTowers(agent_team), self.agent, BULLETRANGE):
            self.agent.changeState(AttackTower)

        if isEnemyBaseAround(self.agent.world.getEnemyBases(agent_team), self.agent, BULLETRANGE):
            self.agent.changeState(AttackBase)

        if isEnemyNPCAround(self.agent.getVisibleType(MOBAAgent), self.agent.position, self.agent.getTeam(),
                            BULLETRANGE):
            self.agent.changeState(AttackMinion)

        if self.path is None:
            self.path = []

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

        if self.target is None:
            return

        if not rayTest(agent_pos, self.target, world, self.agent):
            # print "Collision Ahead!"
            network = []
            for edge in nav.pathnetwork:
                if not rayTraceWorld(edge[0], edge[1], world.getLinesWithoutBorders()):
                    network.append(edge)
            network_agent_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, world.getLinesWithoutBorders())
            network_dest_pos = findClosestUnobstructed(self.destination, nav.pathnodes, world.getLinesWithoutBorders())
            new_path, closed_nodes = astar(network_agent_pos, network_dest_pos, network)
            if len(new_path) > 0:
                # print "Found new path"
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
