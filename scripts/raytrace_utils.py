#!/usr/bin/env python

"""
Implements the raytracing functionalities necessacry to check reachablity and compute gain
All computations for raytracing is done using cell coordinate system and not metric measurements
"""


import math

class RayTrace:

    def __init__(self, cost_map=None, sensor_config={'range':10}):

        self.cost_map = cost_map #numpy array
        self.cost_map_origin = None
        self.sensor_config = sensor_config


    def set_map(self,gmap, orign):
        self.cost_map = gmap
        self.cost_map_origin = orign
    
    def is_obstacle(self, cell_coordinate):
        # cell_coordinate is the point on the map in num_cells , (100,100) means cell (100,100) not meters
        print(self.cost_map[cell_coordinate[0],cell_coordinate[1]])
        if self.cost_map[cell_coordinate[0],cell_coordinate[1]] > 0:
            return True
        else:
            return False


    def cast_ray(self, start_coord, end_coord):

        ray = []
        collision = False
        collision_dist = None
        x0,y0 = start_coord
        x1,y1 = end_coord
        """
        generate integer coordinates on the line from (x0, y0) to (x1, y1).
        Input coordinates should be integers.
        The result will contain both the start and the end point.
        """

        dx = x1 - x0
        dy = y1 - y0

        xsign = 1 if dx > 0 else -1
        ysign = 1 if dy > 0 else -1

        dx = abs(dx)
        dy = abs(dy)

        if dx > dy:
            xx, xy, yx, yy = xsign, 0, 0, ysign
        else:
            dx, dy = dy, dx
            xx, xy, yx, yy = 0, ysign, xsign, 0

        D = 2*dy - dx
        y = 0
        # print("X :",dx, " Y : ", dy)
        for x in range(dx + 1):
            position = (x0 + x*xx + y*yx, y0 + x*xy + y*yy)
            ray.append(position)
            if not self.is_obstacle(position):
                ray.append(position)
            else:
                collision = True
                collision_dist = math.sqrt((x1-x0)**2 + (y1-y0)**2) # TODO : compute collision distance properly
            if D >= 0:
                y += 1
                D -= 2*dx
            D += 2*dy
            if collision:
                break    
    
        return collision, collision_dist ,ray #cells status in ray needed ??






if __name__ == '__main__':

    r = RayTrace()
    pts = r.cast_ray((100,100), (250,250))
    print(pts)

