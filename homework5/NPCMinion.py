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


class NPCMinion(Minion):

    def __init__(self, position, orientation, world, image=NPC, speed=SPEED, viewangle=360, hitpoints=HITPOINTS,
                 firerate=FIRERATE, bulletclass=SmallBullet):
        Minion.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass)
        self.states = [Idle, MoveToEnemyTower, MoveToEnemyBase, AttackMinion, AttackTower, AttackBase, GatherAtTower]

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
        npc = self.agent.world.getNPCsForTeam(self.agent.getTeam())
        self.agent.changeState(MoveToEnemyTower)
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

def sub_vect(vect1, vect2):
    res = (vect1[0] - vect2[0], vect1[1] - vect2[1])
    return res


def length_vect(vect):
    comp_1 = pow(vect[0], 2)
    comp_2 = pow(vect[1], 2)
    return math.sqrt(comp_1 + comp_2)


def div_scalar_vect(vect, scalar):
    res = (vect[0] / scalar, vect[1] / scalar)
    return res


def getOrthoUnitVect(vect):
    origin_vector = sub_vect(vect[1], vect[0])
    if origin_vector[0] == 0:
        unit_ortho_vect = (1, 0)
        return unit_ortho_vect
    elif origin_vector[1] == 0:
        unit_ortho_vect = (0, 1)
        return unit_ortho_vect
    comp_x = - (float(origin_vector[1]) / float(origin_vector[0]))
    comp_y = 1.0
    ortho_vect = (comp_x, comp_y)
    magnitude = length_vect(ortho_vect)
    unit_ortho_vect = div_scalar_vect(ortho_vect, magnitude)

    return unit_ortho_vect


def scale_vect(vect, scalar):
    res = (vect[0] * scalar, vect[1] * scalar)
    return res


def add_vect(vect1, vect2):
    res = (vect1[0] + vect2[0], vect1[1] + vect2[1])
    return res


def parallel_paths(candidate, radius):
    ortho_vect = getOrthoUnitVect(candidate)
    ortho_vect = scale_vect(ortho_vect, radius)
    neg_ortho_vect = (-ortho_vect[0], -ortho_vect[1])

    start_1 = add_vect(candidate[0], ortho_vect)
    start_2 = add_vect(candidate[0], neg_ortho_vect)

    end_1 = add_vect(candidate[1], ortho_vect)
    end_2 = add_vect(candidate[1], neg_ortho_vect)

    path_1 = (start_1, end_1)
    path_2 = (start_2, end_2)

    return path_1, path_2


def intersectWithObstacles(candidate, obstacles, radius):
    for obs in obstacles:
        for line in obs.getLines():
            path_1, path_2 = parallel_paths(candidate, radius)
            intersect_path = rayTrace(candidate[0], candidate[1], line)
            intersect_path_1 = rayTrace(path_1[0], path_1[1], line)
            intersect_path_2 = rayTrace(path_2[0], path_2[1], line)

            if intersect_path or intersect_path_1 or intersect_path_2:
                return True

    return False


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


def rayTest(source, dest, world_lines, agent):
    valid = True
    for line in world_lines:
        point1 = line[0]
        point2 = line[1]
        dist1 = minimumDistance((source, dest), point1)
        dist2 = minimumDistance((source, dest), point2)
        dist3 = minimumDistance(line, source)
        dist4 = minimumDistance(line, dest)
        intersect = rayTraceNoEndpoints(source, dest, line)
        if intersect or dist1 <= agent.getMaxRadius() or dist2 <= agent.getMaxRadius() or dist3 <= agent.getMaxRadius() or dist4 <= agent.getMaxRadius():
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


def getNearBullets(bullets, agent):
    near_bullets = []
    agent_pos = agent.position
    for bullet in bullets:
        dist = euclideanDist(bullet.position, agent_pos)
        if dist <= BULLETRANGE:
            near_bullets.append(bullet)
    return near_bullets


def locationOccupied(agent_pos, agent_radius, npc_list):
    for npc in npc_list:
        dist = euclideanDist(agent_pos, npc.position)
        if dist < agent_radius and dist != 0:
            return True
    return False


def isBulletNear(bullets, agent):
    agent_pos = agent.position
    for bullet in bullets:
        dist = euclideanDist(bullet.position, agent_pos)
        if dist <= 50:
            return True
    return False


def dot_prod(vect1, vect2):
    res = 0
    vect1_a, vect1_b = vect1
    vect2_a, vect2_b = vect2
    vect1 = (vect1_b[0] - vect1_a[0], vect1_b[1] - vect1_a[1])
    vect2 = (vect2_b[0] - vect2_a[0], vect2_b[1] - vect2_a[1])
    for i in range(len(vect1)):
        res += vect1[i] * vect2[i]
    return res


def mag(vect1):
    return euclideanDist(vect1[0], vect1[1])


def vectorAngle(agent_position, enemy_pos, new_pos):
    vect1 = (agent_position, enemy_pos)
    vect2 = (agent_position, new_pos)
    dot_vect = dot_prod(vect1, vect2)
    mag1 = mag(vect1)
    mag2 = mag(vect2)
    cos_theta = dot_vect / (mag1 * mag2)
    theta_rad = math.acos(cos_theta)
    theta_degree = theta_rad * 180 / math.pi
    return theta_degree


def getRandomFormLocation(self, enemy_pos):
    angle = (math.pi * 2) * random.random()
    new_agent_x = enemy_pos[0] + (math.cos(angle) * BULLETRANGE / 2)
    new_agent_y = enemy_pos[1] + (math.sin(angle) * BULLETRANGE / 2)
    new_pos = (new_agent_x, new_agent_y)
    dist = euclideanDist(self.agent.position, new_pos)
    form_location = None
    if rayTest(self.agent.position, new_pos, self.agent.world.getLines(), self.agent) and dist > 0:
        form_location = (new_agent_x, new_agent_y)
    return form_location


def updateNewFormLocation(self):
    if len(self.enemy_towers) > 0:
        enemy_pos = self.enemy_towers[0].position
        if self.form_location:
            dist = euclideanDist(self.agent.position, self.form_location)
            if dist > self.agent.getMaxRadius():
                self.agent.moveToTarget(self.form_location)
            else:
                self.form_location = None
        else:
            self.form_location = getRandomFormLocation(self, enemy_pos)


class AttackTower(State):
    def __init__(self, agent, args=[]):
        super(AttackTower, self).__init__(agent, args)
        self.enemy_towers = []
        self.form_location = None

    def enter(self, oldstate):
        # print "Attack Tower"
        pass

    def execute(self, delta=0):
        State.execute(self, delta)
        agent_team = self.agent.getTeam()
        self.enemy_towers = getEnemyTowers(self.agent.world.getEnemyTowers(agent_team),
                                           self.agent.position, BULLETRANGE)
        if len(self.enemy_towers) > 0:
            enemy_pos = self.enemy_towers[0].position
            if enemy_pos:
                updateNewFormLocation(self)
                self.agent.turnToFace(enemy_pos)
                self.agent.shoot()
        else:
            self.agent.changeState(MoveToEnemyTower)

    def parseArgs(self, args):
        pass


class AttackBase(State):
    def __init__(self, agent, args=[]):
        super(AttackBase, self).__init__(agent, args)
        self.enemy_bases = []
        self.form_location = None

    def enter(self, oldstate):
        # print "Attack Base"
        self.agent.stopMoving()

    def execute(self, delta=0):
        State.execute(self, delta)
        agent_team = self.agent.getTeam()
        self.enemy_bases = self.agent.world.getEnemyBases(agent_team)
        if len(self.enemy_bases) > 0:
            enemy_pos = self.enemy_bases[0].position
            if enemy_pos:
                if self.form_location:
                    dist = euclideanDist(self.agent.position, self.form_location)
                    if dist > self.agent.getMaxRadius():
                        self.agent.moveToTarget(self.form_location)
                    else:
                        self.form_location = None
                else:
                    self.form_location = getRandomFormLocation(self, enemy_pos)
                self.agent.turnToFace(enemy_pos)
                self.agent.shoot()
        else:
            self.agent.changeState(MoveToEnemyBase)

    def parseArgs(self, args):
        pass


class AttackMinion(State):
    def __init__(self, agent, args=[]):
        super(AttackMinion, self).__init__(agent, args)
        self.enemy_minions = []
        self.form_location = None

    def enter(self, oldstate):
        # print "Attack Minion"
        self.agent.stopMoving()

    def execute(self, delta=0):
        State.execute(self, delta)

        visibles = self.agent.getVisibleType(MOBAAgent)
        agent_team = self.agent.getTeam()
        self.enemy_minions = getEnemies(visibles, self.agent.position, agent_team, BULLETRANGE)

        if isEnemyTowerAround(self.agent.world.getEnemyTowers(agent_team), self.agent, BULLETRANGE):
            self.agent.changeState(AttackTower)
            return

        if not self.enemy_minions:
            self.enemy_minions = []

        if len(self.enemy_minions) > 0:
            enemy_pos = self.enemy_minions[0].position
            if enemy_pos:
                if self.form_location:
                    dist = euclideanDist(self.agent.position, self.form_location)
                    if dist > self.agent.getMaxRadius():
                        self.agent.moveToTarget(self.form_location)
                    else:
                        self.form_location = None
                else:
                    self.form_location = getRandomFormLocation(self, enemy_pos)
                self.agent.turnToFace(enemy_pos)
                self.agent.shoot()
        else:
            self.agent.changeState(MoveToEnemyTower)

    def parseArgs(self, args):
        pass


def getAliveTower(enemy_towers):
    for tower in enemy_towers:
        if tower.hitpoints > 0:
            return tower
    return None


def allEnemyTowerDead(enemy_towers):
    return len(enemy_towers) == 0


def findPath(self):
    nav = self.agent.navigator
    agent_pos = self.agent.position
    world = self.agent.world
    network_agent_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, world.getLinesWithoutBorders())
    network_dest_pos = findClosestUnobstructed(self.destination, nav.pathnodes, world.getLinesWithoutBorders())
    new_path = []
    if network_dest_pos and network_agent_pos:
        network = []
        for edge in nav.pathnetwork:
            if not rayTraceWorldNoEndPoints(edge[0], edge[1], world.getLinesWithoutBorders()):
                network.append(edge)
        new_path, closed_nodes = astar(network_agent_pos, network_dest_pos, network)
    return new_path


class MoveToEnemyTower(State):
    def __init__(self, agent, args=[]):
        super(MoveToEnemyTower, self).__init__(agent, args)
        self.path = []
        self.destination = None
        self.curr_pos = None
        self.target = None
        self.selected_tower = None

    def enter(self, oldstate):
        # print "Move"
        State.enter(self, oldstate)
        enemy_towers = self.agent.world.getEnemyTowers(self.agent.getTeam())

        if allEnemyTowerDead(enemy_towers):
            self.agent.changeState(MoveToEnemyBase)
        else:
            self.selected_tower = getAliveTower(enemy_towers)
            self.destination = self.selected_tower.position
            self.path = findPath(self)

        if self.path is None:
            self.path = []

        if len(self.path) > 0:
            self.target = self.path[0]
            self.agent.navigator.destination = self.path[0]
            self.path = self.path[1:len(self.path)]

    def execute(self, delta=0):
        State.execute(self, delta)
        enemy_towers = self.agent.world.getEnemyTowers(self.agent.getTeam())
        agent_team = self.agent.getTeam()
        nav = self.agent.navigator
        agent_pos = self.agent.position
        world = self.agent.world

        if allEnemyTowerDead(enemy_towers):
            self.agent.changeState(MoveToEnemyBase)
            return

        if self.selected_tower.hitpoints == 0:
            self.destination = None

        if self.destination is None:  # find path to a tower
            self.destination = getAliveTower(enemy_towers).position
            self.path = findPath(self)

        if isEnemyTowerAround(self.agent.world.getEnemyTowers(agent_team), self.agent, BULLETRANGE):
            self.agent.changeState(AttackTower)
            return

        if isEnemyNPCAround(self.agent.getVisibleType(MOBAAgent), self.agent.position, self.agent.getTeam(),
                            BULLETRANGE):
            self.agent.changeState(AttackMinion)
            return

        if self.path is None:
            self.path = []

        # if len(self.path) == 0:
        #     new_path = findPath(self)
        #     if len(new_path) > 0:
        #         self.path = new_path
        #         self.target = self.path[0]
        #         self.agent.navigator.destination = self.path[0]
        #         self.path = self.path[1:len(self.path)]

        if self.target is None:
            return

        if not rayTest(agent_pos, self.target, world.getLinesWithoutBorders(), self.agent):
            new_path = findPath(self)
            if len(new_path) > 0:
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
            # network_agent_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, world.getLinesWithoutBorders())
            # network_dest_pos = findClosestUnobstructed(self.destination, nav.pathnodes, world.getLinesWithoutBorders())
            # new_path = []
            # if network_dest_pos and network_agent_pos:
            #     network = []
            #     for edge in nav.pathnetwork:
            #         if not rayTraceWorld(edge[0], edge[1], world.getLinesWithoutBorders()):
            #             network.append(edge)
            #     new_path, closed_nodes = astar(network_agent_pos, network_dest_pos, network)
            self.path = findPath(self)

        if self.path is None:
            self.path = []

        if len(self.path) > 0:
            self.target = self.path[0]
            self.agent.navigator.destination = self.path[0]
            self.path = self.path[1:len(self.path)]

    def execute(self, delta=0):
        State.execute(self, delta)
        enemy_bases = self.agent.world.getEnemyBases(self.agent.getTeam())
        agent_team = self.agent.getTeam()
        nav = self.agent.navigator
        agent_pos = self.agent.position
        world = self.agent.world

        if len(enemy_bases) == 0:
            self.agent.changeState(Idle)
            return

        if not self.destination:
            return

        if isEnemyBaseAround(self.agent.world.getEnemyBases(agent_team), self.agent, BULLETRANGE):
            self.agent.changeState(AttackBase)
            return

        if isEnemyNPCAround(self.agent.getVisibleType(MOBAAgent), self.agent.position,
                            self.agent.getTeam(), BULLETRANGE):
            self.agent.changeState(AttackMinion)
            return

        if self.path is None:
            self.path = []

        if len(self.path) == 0:
            # network = []
            # for edge in nav.pathnetwork:
            #     if not rayTraceWorld(edge[0], edge[1], world.getLinesWithoutBorders()):
            #         network.append(edge)
            # network_agent_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, world.getLinesWithoutBorders())
            # network_dest_pos = findClosestUnobstructed(self.destination, nav.pathnodes, world.getLinesWithoutBorders())
            # new_path = []
            # if network_dest_pos and network_agent_pos:
            #     new_path, closed_nodes = astar(network_agent_pos, network_dest_pos, network)
            new_path = findPath(self)
            if len(new_path) > 0:
                self.path = new_path
                self.target = self.path[0]
                self.agent.navigator.destination = self.path[0]
                self.path = self.path[1:len(self.path)]

        if self.target is None:
            return

        if not rayTest(agent_pos, self.target, world.getLinesWithoutBorders(), self.agent):
            # print "Collision Ahead!"
            # network = []
            # for edge in nav.pathnetwork:
            #     if not rayTraceWorld(edge[0], edge[1], world.getLinesWithoutBorders()):
            #         network.append(edge)
            # network_agent_pos = findClosestUnobstructed(agent_pos, nav.pathnodes, world.getLinesWithoutBorders())
            # network_dest_pos = findClosestUnobstructed(self.destination, nav.pathnodes, world.getLinesWithoutBorders())
            # new_path = []
            # if network_dest_pos and network_agent_pos:
            #     new_path, closed_nodes = astar(network_agent_pos, network_dest_pos, network)
            new_path = findPath(self)
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


class GatherAtTower(State):
    def __init__(self, agent, args=[]):
        super(GatherAtTower, self).__init__(agent, args)
        self.towers = []
        self.selected_tower = None
        self.gather_pos = None
        self.npc = []

    def enter(self, oldstate):
        State.enter(self, oldstate)
        # stop moving
        self.towers = self.agent.world.getTowersForTeam(self.agent.getTeam())
        self.npc = self.agent.world.getNPCsForTeam(self.agent.getTeam())
        indx = random.randrange(0, len(self.towers))
        if len(self.towers) > 0:
            self.selected_tower = self.towers[indx]
            self.gather_pos = findClosestUnobstructed(self.selected_tower.position,
                                                      self.agent.navigator.pathnodes,
                                                      self.agent.world.getLines())

    def execute(self, delta=0):
        State.execute(self, delta)
        ### YOUR CODE GOES BELOW HERE ###
        count = 0
        for npc in self.npc:
            dist = euclideanDist(self.agent.position, npc.position)
            if dist <= BULLETRANGE:
                count += 1
        if self.gather_pos and count <= 5:
            self.agent.moveToTarget(self.gather_pos)
        else:
            self.agent.changeState(MoveToEnemyBase)

        ### YOUR CODE GOES ABOVE HERE ###
        return None
