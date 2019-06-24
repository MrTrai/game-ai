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

from collections import deque
from constants import *
from utils import *
from core import *
from itertools import combinations
import copy


def getPolySides(poly):
    sides = []
    length = len(poly)
    for i in range(length):
        sides.append(
            (poly[i], poly[(i + 1) % length])
        )
    return sides


def triangleArea(triangle):
    pt1, pt2, pt3 = triangle
    pt1_x, pt1_y = pt1  # A
    pt2_x, pt2_y = pt2  # B
    pt3_x, pt3_y = pt3  # C
    total = (pt1_x * (pt2_y - pt3_y)) + (pt2_x * (pt3_y - pt1_y)) + (pt3_x * (pt1_y - pt2_y))
    area = abs(total / 2)
    return area


def polyArea(poly):
    if len(poly) == 3:
        return triangleArea(poly)
    points = convex_hull(poly)
    triangles = []
    area = 0
    for i in range(1, len(points) - 1):
        triangle = [points[0], points[i], points[i + 1]]
        triangles.append(triangle)
    for triangle in triangles:
        area += triangleArea(triangle)
    return area


def isGoal(percent):
    return percent >= 99


def sameEdge(tri_edge, poly_edge):
    tri_edge_x, tri_edge_y = tri_edge
    poly_edge_x, poly_edge_y = poly_edge
    return (tri_edge_x == poly_edge_x and tri_edge_y == poly_edge_y) or \
           (tri_edge_x == poly_edge_y and tri_edge_y == poly_edge_x)


def triangleCentroid(triangle):
    x = 0
    y = 0
    for pt in triangle:
        x += pt[0]
        y += pt[1]
    centroid_x = x / len(triangle)
    centroid_y = y / len(triangle)
    return (centroid_x, centroid_y)


def generateTriangles(world):
    points = world.getPoints()
    triangles = list(combinations(points, 3))
    return triangles


def exhaustedSearch(curr_config, triangles):
    polys, explored_index, num_big_polys = curr_config
    return len(explored_index) == len(triangles)


def betterConfigThan(curr_config, best_config):
    polys, explored_index, num_big_polys = curr_config
    best_polys, best_explored_index, best_num_big_polys = best_config
    return len(polys) > len(best_polys)


def canAdd(triangle, polys):
    tri_edges = list(combinations(triangle, 2))
    for poly in polys:
        poly_edges = list(combinations(poly, 2))
        for tri_edge in tri_edges:
            pt1, pt2 = tri_edge
            for poly_edge in poly_edges:
                if not sameEdge(tri_edge, poly_edge):
                    intersect = rayTraceNoEndpoints(pt1, pt2, poly_edge)
                    if intersect:
                        return False
    return True


def countBigPolys(polys):
    count = 0
    for poly in polys:
        if len(poly) >= 4:
            count += 1
    return count


def nextStates(curr_config, triangles):
    states = []
    polys, explored_index, num_big_polys = curr_config
    overlap = []
    if len(explored_index) == len(triangles):
        return []
    for i in range(len(triangles)):
        if i not in explored_index:
            new_state = None
            if canAdd(triangles[i], polys):
                new_polys = copy.copy(polys)
                new_polys.append(triangles[i])
                # new_polys = mergePoly2(new_polys)
                new_polys, new_num_big_polys = mergePoly(new_polys)
                new_explored_index = copy.copy(explored_index)
                new_explored_index.append(i)
                new_num_big_polys = countBigPolys(new_polys)
                new_state = (new_polys, new_explored_index, new_num_big_polys)
            else:
                overlap.append(i)
            if new_state:
                states.append(new_state)
    for state in states:
        for overlap_poly in overlap:
            state[1].append(overlap_poly)
    return states


def convex_hull2(points):
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    points = sorted(set(points))
    curr = points[0]
    convex = [points[0]]

    while len(convex) < len(points):
        for p1 in points:
            if p1 not in convex and p1 != curr:
                candidate = True
                for p2 in points:
                    if p2 not in convex and p2 != p1 and p2 != curr and cross(curr, p1, p2) > 0:
                        candidate = False
                if candidate:
                    convex.append(p1)
                    curr = p1
                    break

    return convex


def convex_hull(points):
    """Computes the convex hull of a set of 2D points.

    Input: an iterable sequence of (x, y) pairs representing the points.
    Output: a list of vertices of the convex hull in counter-clockwise order,
      starting from the vertex with the lexicographically smallest coordinates.
    Implements Andrew's monotone chain algorithm. O(n log n) complexity.
    """

    # Sort the points lexicographically (tuples are compared lexicographically).
    # Remove duplicates to detect the case we have just one unique point.
    points = sorted(set(points))

    # Boring case: no points or a single point, possibly repeated multiple times.
    if len(points) <= 1:
        return points

    # 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
    # Returns a positive value, if OAB makes a counter-clockwise turn,
    # negative for clockwise turn, and zero if the points are collinear.
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Build lower hull
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenation of the lower and upper hulls gives the convex hull.
    # Last point of each list is omitted because it is repeated at the beginning of the other list.
    return lower[:-1] + upper[:-1]


def getObsArea(world):
    obstacles = world.getObstacles()
    area = 0
    for obs in obstacles:
        area += polyArea(obs.getPoints())
    return area


def percent_coverage(polys, world_area, obs_area):
    total_poly_area = 0
    for poly in polys:
        total_poly_area += polyArea(poly)
    total_area = (total_poly_area + obs_area) * 1.0
    percent = total_area * 100.0 / (world_area * 1.0)
    return percent


def generatePolys(triangles, world):
    visit = []
    # state = (explored polys config, indices of unexplored polys, number of polys > 4 sides)
    length = len(triangles)
    dim = world.getDimensions()
    world_area = dim[0] * dim[1]
    obs_area = getObsArea(world)
    for i in range(length):
        state = ([triangles[i]], [i], 0)
        visit.append(state)

    best_config = None
    step = 0
    limit = 1000
    while visit and step < limit:
        step += 1
        curr_config = visit.pop()
        polys, explored_index, num_big_polys = curr_config

        curr_coverage_percent = percent_coverage(polys, world_area, obs_area)
        if isGoal(curr_coverage_percent):
            if best_config is None:
                best_config = (curr_config, curr_coverage_percent)
            else:
                best_state, best_coverage = best_config
                best_polys, best_explored_index, best_big_polys = best_config[0]
                print best_big_polys
                if curr_coverage_percent > best_coverage or num_big_polys > best_big_polys:
                    best_config = (curr_config, curr_coverage_percent)
        next_states = nextStates(curr_config, triangles)
        for state in next_states:
            visit.append(state)

    return best_config[0]


def crossObstacles(tri_lines, obs_lines):
    for tri_line in tri_lines:
        pt1, pt2 = tri_line
        for obs_line in obs_lines:
            isSameEdge = sameEdge(tri_line, obs_line)
            if not isSameEdge:
                intersect = rayTraceNoEndpoints(pt1, pt2, obs_line)
                if intersect:
                    return True
    return False


def insideObstacles(tri, world):
    obstacles = world.getObstacles()
    pt1, pt2, pt3 = tri
    for obs in obstacles:
        corners = []
        obs_corners = obs.getPoints()
        if pt1 in obs_corners:
            corners.append(pt1)
        if pt2 in obs_corners:
            corners.append(pt2)
        if pt3 in obs_corners:
            corners.append(pt3)
        if len(corners) == 3:
            return True
        if len(corners) == 2:
            obs_lines = obs.getLines()
            line = (corners[0], corners[1])
            line_inv = (corners[1], corners[0])
            return not ((line in obs_lines) or (line_inv in obs_lines))
    return False


def characterWidthSatisfy(tri_lines, world, agent):
    obstacles = world.getObstacles()
    for obs in obstacles:
        obs_corners = obs.getPoints()
        for obs_corner in obs_corners:
            for tri_line in tri_lines:
                if minimumDistance(tri_line, obs_corner) <= agent.getMaxRadius():
                    return False
    return True


def sanityCheckTriangles(triangles, world, agent):
    result = []
    obs_lines = []
    for obs in world.getObstacles():
        for l in obs.getLines():
            obs_lines.append(l)
    for tri in triangles:
        tri_lines = list(combinations(tri, 2))
        cross = crossObstacles(tri_lines, obs_lines)
        inside = insideObstacles(tri, world)
        if not cross and not inside:
            result.append(tri)
    return result


def polyEdges(poly):
    edges = []
    length = len(poly)
    for i in range(0, length):
        start = poly[i]
        end = poly[(i + 1) % length]
        edges.append((start, end))
    return edges


def generatePathEdges2(polys, world, agent):
    lines = []

    for poly in polys:
        edges = polyEdges(poly)
        nodes = []
        for edge in edges:
            mid = midPoint(edge[0], edge[1])
            nodes.append(mid)
        for i in range(len(nodes)):
            node1 = nodes[i]
            for j in range(i + 1, len(nodes)):
                node2 = nodes[j]
                path_edge = (node1, node2)
                valid = True
                for obs in world.getObstacles():
                    obs_edges = obs.getLines()
                    for obs_edge in obs_edges:
                        point1 = obs_edge[0]
                        point2 = obs_edge[1]
                        min_dist1 = minimumDistance(path_edge, point1)
                        min_dist2 = minimumDistance(path_edge, point2)
                        intersect = rayTrace(point1, point2, path_edge)

                        if min_dist1 <= agent.getMaxRadius() or min_dist2 <= agent.getMaxRadius() or intersect:
                            valid = False

                if valid:
                    lines.append((node1, node2))
    return lines

def generatePathEdges(pathnodes, world, agent):
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


def generatePathNodes(polys):
    nodes = []
    for poly in polys:
        centroid = triangleCentroid(poly)
        nodes.append(centroid)

    return nodes


def midPoint(p1, p2):
    p1_x, p1_y = p1
    p2_x, p2_y = p2
    mid_x = (p1_x + p2_x) / 2
    mid_y = (p1_y + p2_y) / 2
    return (mid_x, mid_y)


def generatePathNodes2(polys):
    length = len(polys)
    nodes = []
    for i in range(length):
        for j in range(i + 1, length):
            adjacent = polygonsAdjacent(polys[i], polys[j])
            if adjacent:
                for k in range(len(adjacent)):
                    for m in range(k + 1, len(adjacent)):
                        p1, p2 = adjacent[k], adjacent[m]
                        mid = midPoint(p1, p2)
                        nodes.append(mid)

    return nodes


def merge(poly1, poly2, adjacent):
    # find common point
    adj1 = adjacent[0]
    adj2 = adjacent[1]

    poly1 = deque(poly1)
    poly2 = deque(poly2)

    poly1.remove(adj2)
    while poly1[0] != adj1:
        poly1.rotate(1)

    poly2.remove(adj1)
    while poly2[0] != adj2:
        poly2.rotate(1)

    poly1 += poly2
    points = poly1

    return tuple(points)


def matchPoly(p, poly2):
    flag = True
    for pt1 in p:
        if pt1 not in poly2:
            flag = False
    return flag


def isGoalMesh(changed):
    return not changed


def nextStatesMerge(curr_state):
    curr_polys, curr_polys_count = curr_state
    length = len(curr_polys)
    states = []
    for i in range(length):
        for j in range(i + 1, length):
            poly1 = curr_polys[i]
            poly2 = curr_polys[j]
            adjacent = polygonsAdjacent(poly1, poly2)
            if adjacent:
                merged_poly = merge(poly1, poly2, adjacent)
                if isConvex(merged_poly):
                    new_polys = [p for p in curr_polys if p != poly1 and p != poly2]
                    new_polys.append(merged_poly)
                    state = (new_polys, length - 1)
                    states.append(state)
    return states


def mergePoly(polys):
    # state = (merged meshes, triangle count, changed boolean)
    visit = []
    length = len(polys)
    for i in range(length):
        for j in range(i + 1, length):
            poly1 = polys[i]
            poly2 = polys[j]
            adjacent = polygonsAdjacent(poly1, poly2)
            if adjacent:
                merged_poly = merge(poly1, poly2, adjacent)
                if isConvex(merged_poly):
                    new_polys = [p for p in polys if p != poly1 and p != poly2]
                    new_polys.append(merged_poly)
                    state = (new_polys, length - 1)
                    visit.append(state)

    best = None
    step = 0
    limit = 800
    while visit and step < limit:
        step += 1
        curr_state = visit.pop()
        curr_polys, curr_polys_count = curr_state

        if best is None:
            best = curr_state
        else:
            best_polys, best_polys_count = best
            if best_polys_count > curr_polys_count:
                best = curr_state

        for new_state in nextStatesMerge(curr_state):
            visit.append(new_state)

    if best is None:
        return polys, 0
    return best


def mergePoly2(polys):
    nodes = [p for p in polys]
    changed = True
    while changed:
        changed = False
        for poly1 in nodes:
            for poly2 in nodes:
                if poly1 == poly2:
                    continue
                adjacent = polygonsAdjacent(poly1, poly2)
                if adjacent:
                    merged = merge(poly1, poly2, adjacent)
                    if isConvex(merged):
                        try:
                            nodes.remove(poly1)
                        except ValueError:
                            print "Try to remove poly1"
                            pass
                        try:
                            nodes.remove(poly2)
                        except ValueError:
                            print "Try to remove poly2"
                            pass
                        nodes.append(merged)
                        changed = True
                        break
            if changed:
                break

    return nodes


def myCreatePathNetwork(world, agent=None):
    triangles = generateTriangles(world)
    triangles = sanityCheckTriangles(triangles, world, agent)
    polys, explored_index, num_big_polys = generatePolys(triangles, world)
    print "Explored: ", len(explored_index)
    print "Triangle: ", len(triangles)
    nodes = generatePathNodes2(polys)
    edges = generatePathEdges(nodes, world, agent)
    return nodes, edges, polys
