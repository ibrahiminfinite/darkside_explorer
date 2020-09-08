#!/usr/bin/env python
"""
Implements the raytracing functionalities necessacry to check reachablity and compute gain
"""

#NOTE :
# 1. implment cast ray function which returns bool:collision, float : collision_dist, tuple : end_point



class RayTrace:

    def __init__(self, cost_map, sensor_config):

        self.cost_map = cost_map
        self.sensor_config = sensor_config

    
    def is_obstacle(self, position):
        pass

    def cast_ray(self, start_position, end_position):

        ray = []
        collision = False
        collision_dist = None
        x0,y0 = start_position
        x1,y1 = end_position
        """generate integer coordinates on the line from (x0, y0) to (x1, y1).
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

        for x in range(dx + 1):
            position = (x0 + x*xx + y*yx, y0 + x*xy + y*yy)
            if not self.is_obstacle(position):
                ray.append(position)
            else:
                if D >= 0:
                    y += 1
                    D -= 2*dx
            D += 2*dy

    
        return collision, collision_dist ,ray #cells status in ray needed ??

