from math import sqrt, acos, atan2, sin, cos
import math
import numpy as np
import matplotlib.pyplot as plt



def get_intersecting_obstacles(obstacles, point, pos): 
    intersecting_obstacles = {}
    
    for o in obstacles:
        existance, intersection = get_intersecting_points(o, pos, point)
        
        if existance == False: 
            pass
        else: 
            intersecting_obstacles[(tuple(o[0]), o[1])] = intersection
    
    return intersecting_obstacles


def get_stopping_point(obstacles, point, pos, boundary_walls): 
    intersecting_circles = get_intersecting_obstacles(obstacles, point, pos)

    stopping_circle = ()
    stopping_edge = ()

    tangent_circles = []

    stopping_point = []
    if intersecting_circles == {}:
        intersecting_walls = get_intersecting_walls((pos, (point[0], point[1])), boundary_walls)
        stopping_point, stopping_edge = get_wall_stopping_point(intersecting_walls, (pos, point), boundary_walls, pos)
        
    else: 
        closest_distance = math.inf
        for circle in intersecting_circles:
            if intersecting_circles[circle][0] == intersecting_circles[circle][1]:
                tangent_circles.append(circle)
            else:
                for intersect in intersecting_circles[circle]:
                    if get_distance(pos, intersect) < closest_distance:
                        if round(get_angle(pos, intersect), 0) == round(get_angle(pos, point), 0):
                            closest_distance = get_distance(pos, intersect)
                            stopping_point = intersect
                            stopping_circle = circle
        
        if stopping_point == []:
            intersecting_walls = get_intersecting_walls((pos, (point[0], point[1])), boundary_walls)
            stopping_point, stopping_edge = get_wall_stopping_point(intersecting_walls, (pos, point), boundary_walls, pos)
    
    return stopping_point, stopping_circle, stopping_edge, tangent_circles
           

def get_intersecting_points(circle, pos, point): 
    # ray is drawn from pos to point
    # circle has [x, y], r form
    
    m = (point[1] - pos[1])/(point[0] + 1e-11 - pos[0])
    c = (pos[1]*point[0] - pos[0]*point[1])/(point[0] + 1e-11 - pos[0])
    
    a = circle[0][0]
    b = circle[0][1]
    r = circle[1]
    
    A = m**2 + 1
    B = 2 * m * (c - b) - 2 * a
    C = (c - b)**2 - r**2 + a**2
    
    disc = B**2 - 4 * A * C
    if disc < 0: 
        return False, []
    else: 
        x1 = round((-1*B + disc**0.5)/(2 * A), 2)
        x2 = round((-1*B - disc**0.5)/(2 * A), 2)
        
        y1 = round(m*x1 + c, 2)
        y2 = round(m*x2 + c, 2)
        
        return True, [(x1, y1), (x2, y2)]


def get_distance(p1, p2): 
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**(1/2)

# p1 should always be pos
def get_angle(pos, p2): 
    if p2[1] == pos[1]: 
        if pos[0] <= p2[0]:
            return 0
        else:
            return 180
    
    if p2[0] == pos[0]: 
        if p2[1] > pos[1]: 
            return 90
        else: 
            return 270
    
    m = (p2[1] - pos[1])/(p2[0] - pos[0])
    
    #1st v/s 3rd 
    if m > 0: 
        #1st
        if p2[1] > pos[1]: 
            return math.degrees(math.atan(m))
        
        #3rd
        else: 
            return 180 + math.degrees(math.atan(m))
        
    #2nd v/s 4th 
    if m < 0:
        #4th
        if p2[1] < pos[1]:
            return 360 + math.degrees(math.atan(m))
        #2nd
        else: 
            return 180 + math.degrees(math.atan(m))

def get_intersecting_walls(edge, walls): 
    # edge should always have pos first
    intersecting_walls = []
    A = (edge[1][1] - edge[0][1])/(edge[1][0] + 1e-11 - edge[0][0])
    B = (edge[0][1]*edge[1][0] - edge[0][0]*edge[1][1])/(edge[1][0] + 1e-11 - edge[0][0])
    
    for start in walls: 
        for end in walls[start]: 
            C = (end[1] - start[1])/(end[0] + 1e-11 - start[0])
            D = (start[1]*end[0] - start[0]*end[1])/(end[0] + 1e-11 - start[0])
            if C == A: 
                pass
            else: 
                x = (B - D)/(C - A)
                y = A*x + B
                
                add_to_walls = False
                if point_on_edge((start, end), (x, y)):
                    if edge[0][0] <= edge[1][0]:
                        if x > edge[0][0]: 
                            add_to_walls = True
                    elif edge[0][0] >= edge[1][0]: 
                        if x < edge[0][0]:
                            add_to_walls = True
                if add_to_walls == True: 
                    intersecting_walls.append((start, end))
                    
    return intersecting_walls


# edit to fix error
def get_wall_stopping_point(intersecting_walls, ray, walls, pos): 
    # ray should always have pos first
    stopping_point = ()
    shortest_distance = math.inf

    intersecting_wall = ()

    for wall in intersecting_walls: 
        point = get_intersection(wall, ray)
        found_point = True
        if (round(point[0], 1) == round(wall[0][0], 1) and round(point[1], 1) == round(wall[0][1], 1)) or (round(point[0], 1) == round(wall[1][0], 1) and round(point[1], 1) == round(wall[1][1], 1)):
            point_to_index = round(point[0], 0) + 0.0
            point_to_index_2 = round(point[1], 0) + 0.0
            wall_ends = walls[(point_to_index, point_to_index_2) + (get_angle(pos, (point_to_index, point_to_index_2)),)]
            angle_1 = round(wall_ends[0][2], 2)
            angle_2 = round(wall_ends[1][2], 2)
            curr_angle = round(get_angle(pos, point), 2)
            
            lesser_angle = min(angle_1, angle_2)
            higher_angle = max(angle_1, angle_2)
            
            if not(curr_angle < higher_angle and curr_angle > lesser_angle):
                found_point = False
                
        if found_point:
            this_distance = get_distance(ray[0], point)
            if this_distance < shortest_distance: 
                shortest_distance = this_distance
                stopping_point = point
                intersecting_wall = wall
    return stopping_point, intersecting_wall


def get_third(elem): 
    #print(elem)
    return elem[2]


# getting tangent points in increasing order of angle

def find_tangents(pos, obstacles): 
    # tangent points keeps track which point is associated to which center
    # points keeps track of order of points by angle
    tangent_points = {}
    points = []
    
    for o in obstacles: 
        Px = pos[0]
        Py = pos[1]
        Cx = o[0][0]
        Cy = o[0][1]
        a = o[1]
        
        b = get_distance((Px, Py), (Cx, Cy))
        th = acos(a / b)  # angle theta
        d = atan2(Py - Cy, Px - Cx)  # direction angle of point P from C
        d1 = d + th  # direction angle of point T1 from C
        d2 = d - th  # direction angle of point T2 from C

        T1x = round(Cx + a * cos(d1), 3)
        T1y = round(Cy + a * sin(d1), 3)
        T2x = round(Cx + a * cos(d2), 3)
        T2y = round(Cy + a * sin(d2), 3)
        
        p1 = (T1x, T1y, get_angle(pos, (T1x, T1y)))
        p2 = (T2x, T2y, get_angle(pos, (T2x, T2y)))
        
        points.append(p1)
        points.append(p2)
        
        points.sort(key=get_third)
        
        tangent_points[(Cx, Cy)] = [p1, p2]
        
    
    return tangent_points, points

def point_on_edge(edge, point):  
    if round(point[0], 2) >= round(min(edge[0][0], edge[1][0]), 2) and round(point[0], 2) <= round(max(edge[0][0], edge[1][0]), 2):
        if round(point[1], 2) >= round(min(edge[0][1], edge[1][1]), 2) and round(point[1], 2) <= round(max(edge[0][1], edge[1][1]), 2):
            return True
    return False

def get_intersection(line1, line2): 
    
    A = (line1[1][1] - line1[0][1])/(line1[1][0] + 1e-11 - line1[0][0])
    B = (line1[0][1]*line1[1][0] - line1[0][0]*line1[1][1])/(line1[1][0] + 1e-11 - line1[0][0])
    
    C = (line2[1][1] - line2[0][1])/(line2[1][0] + 1e-11 - line2[0][0])
    D = (line2[0][1]*line2[1][0] - line2[0][0]*line2[1][1])/(line2[1][0] + 1e-11 - line2[0][0])
    
    if (A == C): 
        return None
    x = (D - B)/(A - C)
    y = A*x + B
    
    return (x, y)