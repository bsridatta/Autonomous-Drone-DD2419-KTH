
#!/usr/bin/env python
import json
import matplotlib.pyplot as plt
import numpy as np
import math
from all_grid import *
import matplotlib.pyplot as plt
import math


show_animation = True






class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)






def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    nstart = Node((sx / reso), (sy / reso), 0.0, -1)
    ngoal = Node((gx / reso), (gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map_for_this(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index_for_this(nstart, minx, miny)] = nstart

    while 1:

        print(openset)
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]
        print(c_id)

        #show graph
        if show_animation:  # pragma: no cover
            plt.plot(current.x* reso , current.y*reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)

            n_id = calc_index_for_this(node, minx, miny)

            if n_id in closedset:
                continue
            
            if not verify_node(node, obmap, minx, miny, maxx, maxy):

                continue

            if n_id not in openset:

                openset[n_id] = node  
            else:
                if openset[n_id].cost >= node.cost:

                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry

def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x <= minx:
        return False
    elif node.y <= miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False
    
    


    x_grid = int((node.x - minx)/division)
    y_grid  = int(grid_height-(node.y-miny)/division)
    if obmap[y_grid][x_grid]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):
    

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))




    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)

    obmap = [[False for i in range(int(ywidth))] for i in range(int(xwidth))]
  
    for ix in range(int(xwidth)):
        x = ix + minx
        for iy in range(int(ywidth)):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = float(((iox - x)**2 + (ioy - y)**2)**0.5)
                if d <= vr/reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_obstacle_map_for_this(ox, oy, reso, vr):
    


    minx = x_min
    miny = y_min
    maxx = x_max
    maxy = y_max



    xwidth = grid_width #parts
    ywidth = grid_height

    obmap = [[False for i in range(int(grid_width)+1)] for i in range(int(grid_height)+1)]
    for ix in range(int(grid_width)+1):
        x = ix*division + minx 
        for iy in range(int(grid_height)+1):
            y = (int(grid_height)-iy)*division + miny
            for iox,ioy in zip(ox,oy):
                d = (((iox - x)**2 + (ioy - y)**2)**(0.5))
                if d == 0.0:
            	    obmap[iy][ix]=True
            	    break


    return obmap, minx, miny, maxx, maxy, xwidth, ywidth



def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)

def calc_index_for_this(node,xmin, ymin):
    return (node.y - ymin)/division * (grid_width+1) + (node.x - xmin)/division    


def get_motion_model():

    motion = [[division, 0.0, division],
              [0, division, division],
              [-division, 0, division],
              [0, -division, division],
              [-division, -division,division* math.sqrt(2)],
              [-division, division, division*math.sqrt(2)],
              [division, -division, division*math.sqrt(2)],
              [division, division, division*math.sqrt(2)]]  

    return motion



