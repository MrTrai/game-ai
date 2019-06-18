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

import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *

from math import sqrt
from math import pow


# Creates the path network as a list of lines between all path nodes that are traversable by the agent.
# def sub_vect(vect1, vect2):
#     res = (vect1[0] - vect2[0], vect1[1] - vect2[1])
#     return res
#
#
# def length_vect(vect):
#     comp_1 = pow(vect[0], 2)
#     comp_2 = pow(vect[1], 2)
#     return sqrt(comp_1 + comp_2)
#
#
# def div_scalar_vect(vect, scalar):
#     res = (vect[0] / scalar, vect[1] / scalar)
#     return res
#
#
# def getOrthoUnitVect(candidate):
#     vect = sub_vect(candidate[1], candidate[0])
#     if vect[0] == 0:
#         unit_ortho_vect = (1, 0)
#         return unit_ortho_vect
#     elif vect[1] == 0:
#         unit_ortho_vect = (0, 1)
#         return unit_ortho_vect
#     comp_x = - (float(vect[1]) / float(vect[0]))
#     comp_y = 1.0
#     ortho_vect = (comp_x, comp_y)
#     magnitude = length_vect(ortho_vect)
#     unit_ortho_vect = div_scalar_vect(ortho_vect, magnitude)
#
#     return unit_ortho_vect
#
#
# def scale_vect(vect, scalar):
#     res = (vect[0] * scalar, vect[1] * scalar)
#     return res
#
#
# def add_vect(vect1, vect2):
#     res = (vect1[0] + vect2[0], vect1[1] + vect2[1])
#     return res
#
#
# def parallel_paths(candidate, radius):
#     ortho_vect = getOrthoUnitVect(candidate)
#     ortho_vect = scale_vect(ortho_vect, radius)
#     neg_ortho_vect = (-ortho_vect[0], -ortho_vect[1])
#
#     start_1 = add_vect(candidate[0], ortho_vect)
#     start_2 = add_vect(candidate[0], neg_ortho_vect)
#
#     end_1 = add_vect(candidate[1], ortho_vect)
#     end_2 = add_vect(candidate[1], neg_ortho_vect)
#
#     path_1 = (start_1, end_1)
#     path_2 = (start_2, end_2)
#
#     return path_1, path_2
#
#
# def intersectWithObstacles(candidate, obstacles, radius):
#     for obs in obstacles:
#         for line in obs.getLines():
#             path_1, path_2 = parallel_paths(candidate, radius)
#             intersect_path = rayTrace(candidate[0], candidate[1], line)
#             intersect_path_1 = rayTrace(path_1[0], path_1[1], line)
#             intersect_path_2 = rayTrace(path_2[0], path_2[1], line)
#
#             if intersect_path or intersect_path_1 or intersect_path_2:
#                 return True
#
#     return False

# def myBuildPathNetwork(pathnodes, world, agent=None):
#     lines = []
#     ### YOUR CODE GOES BELOW HERE ###
#     candidates = []
#     for i in range(len(pathnodes) - 1):
#         for j in range(i + 1, len(pathnodes)):
#             n1 = pathnodes[i]
#             n2 = pathnodes[j]
#             candidates.append((n1, n2))
#             candidate = (n1, n2)
#             if not intersectWithObstacles(candidate, world.getObstacles(), agent.getMaxRadius()):
#                 lines.append(candidate)
#     ### YOUR CODE GOES ABOVE HERE ###
#     return lines

def myBuildPathNetwork(pathnodes, world, agent=None):
    lines = []
    ### YOUR CODE GOES BELOW HERE ###
    candidates = []
    for i in range(len(pathnodes) - 1):
        for j in range(i + 1, len(pathnodes)):
            n1 = pathnodes[i]
            n2 = pathnodes[j]
            candidate = (n1, n2)
            candidates.append(candidate)

    for candidate in candidates:
        valid = True
        for obs in world.getObstacles():
            edges = obs.getLines()
            for edge in edges:
                point1 = edge[0]
                point2 = edge[1]
                min_dist1 = minimumDistance(candidate, point1)
                min_dist2 = minimumDistance(candidate, point2)
                intersect = rayTrace(point1, point2, candidate)

                if min_dist1 <= agent.getMaxRadius() or min_dist2 <= agent.getMaxRadius() or intersect:
                    valid = False
        if valid:
            lines.append(candidate)
    ### YOUR CODE GOES ABOVE HERE ###

    return lines


