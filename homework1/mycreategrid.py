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

# Creates a grid as a 2D array of True/False values (True =  traversable). Also returns the dimensions of the grid as a (columns, rows) list.
import math


def isCellIntersectObstacle(obstacles, corners):
    cell_lines = []
    size = len(corners)
    for i in range(size):
        start = corners[i % size]
        end = corners[(i + 1) % size]
        cell_lines.append(
            (start, end)
        )

    for obs in obstacles:
        obs_lines = obs.getLines()
        for obs_line in obs_lines:
            for cell_line in cell_lines:
                intersect = rayTrace(cell_line[0], cell_line[1], obs_line)
                if intersect is not None:
                    print intersect
                    return True

    return False


def isCornerInObstacle(obstacles, corners):
    for corner in corners:
        for obs in obstacles:
            if pointInsidePolygonLines(corner, obs.getLines()):
                return True
    return False


def myCreateGrid(world, cellsize):
    grid = []
    ### YOUR CODE GOES BELOW HERE ###
    screen = world.getDimensions()
    rows = int(math.ceil(screen[1] / cellsize))
    cols = int(math.ceil(screen[0] / cellsize))

    dimensions = (cols, rows)

    for col in range(cols):
        grid.append([True for _ in range(rows)])


    for col in range(cols):
        col_start = int(col * cellsize)
        col_end = int((col + 1) * cellsize)
        for row in range(rows):
            row_start = int(row * cellsize)
            row_end = int((row + 1) * cellsize)
            corner_1 = (col_start, row_start)
            corner_2 = (col_start, row_end)
            corner_3 = (col_end, row_start)
            corner_4 = (col_end, row_end)
            corners = [corner_1, corner_2, corner_3, corner_4]
            if isCellIntersectObstacle(world.getObstacles(), corners) or \
                    isCornerInObstacle(world.getObstacles(), corners):
                grid[col][row] = False

    ### YOUR CODE GOES ABOVE HERE ###
    return grid, dimensions
