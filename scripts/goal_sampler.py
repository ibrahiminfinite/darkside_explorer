#!/usr/bin/env python
"""
samples radial goals with current position for robot as the center
"""

# NOTE :
# 1. Prune the radial samples according  to reachablity (straight line exists without obstacles?)
# 2. Prune again based on robot circumfrence , with pruned goals as centre
# 3. If not goals reduce / increase sampling radius by relaxation factor
# 4. consider distance from last goal and difference in yaw for scoring next goal

import rospy
import math
from raytrace_utils import RayTrace



class Goal:
    
    def __init__(self, gpose, gain):
        self.pose = gpose
        self.gain = gain


class GoalSampler:

    def __init__(self):
        self.angular_step_size = None # determines the number of samples
        self.radial_length = None # in meters
        self.radial_relaxation = None # distance to increase or decrease if no goal found in meters
        self.robot_origin = None
        self.ray_tracer = RayTrace()

    def set_ray_tracer_map(self,gmap, origin):
        self.ray_tracer.set_map(gmap, origin)

    def get_radial_points(self, origin, radius=5, step_size=math.pi/2):
        self.angular_step_size = step_size
        ang = 0 
        radial_points = []
        x, y = origin
        while ang < math.pi*2:  
            # print(ang)                      
            radial_points.append((x + radius*math.cos(ang),y + radius*math.sin(ang)))
            ang += step_size
        
        return radial_points

    def is_reachable_in_straight_line(self, start_pos, end_pos):
        collision, _, _ = self.ray_tracer.cast_ray(start_pos, end_pos)
        return (not collision)



    def get_reachable_points(self, start_coordinates, radial_points):
        #convert point to cell coordinates
        start_x = int((start_coordinates[0] - self.ray_tracer.cost_map_origin[0])/0.05)
        start_y = int((start_coordinates[1] - self.ray_tracer.cost_map_origin[1])/0.05)
        start_coordinates = (start_x, start_y)
        reachable_points = []
        for point in radial_points:
            #convert point to cell coordinates

            x,y = point
            x = int((x- self.ray_tracer.cost_map_origin[0]) / 0.05) # divide by map_resolution
            y = int((y- self.ray_tracer.cost_map_origin[1]) / 0.05)
            cell_coord = (x,y)
            if self.is_reachable_in_straight_line(start_coordinates, end_pos=cell_coord):
                reachable_points.append(point)

        return reachable_points

    

    def generate_scan_at(self,start_coordinates,radius):
        #cell coordinates in meters
        scan = []
        scan_end_points = self.get_radial_points(origin=start_coordinates, radius=radius ,step_size=math.pi/180)
        #convert point to cell coordinates
        start_x = int((start_coordinates[0] - self.ray_tracer.cost_map_origin[0])/0.05)
        start_y = int((start_coordinates[1] - self.ray_tracer.cost_map_origin[1])/0.05)
        start_coord = (start_x, start_y)

        for end_point in scan_end_points:

            x,y = end_point
            x = int((x- self.ray_tracer.cost_map_origin[0]) / 0.05) # divide by map_resolution
            y = int((y- self.ray_tracer.cost_map_origin[1]) / 0.05)
            cell_coord = (x,y)
            _, _, ray = self.ray_tracer.cast_ray(start_coord, cell_coord )
            scan.append(ray)
        return scan

    
    def compute_gain_at(self, origin_coord,radius):

        scan = self.generate_scan_at(origin_coord,radius)
        gain = 0
        for ray in scan:
            for cell_coord in ray:
                if cell_coord[0] >= self.ray_tracer.cost_map.shape[0] or cell_coord[1] >= self.ray_tracer.cost_map.shape[1]:
                 cell_val = -1
                else:
                    cell_val = self.ray_tracer.cost_map[cell_coord[0],cell_coord[1]]
                    if cell_val == -1:
                        gain += 100
                    elif cell_val == 0:
                        gain += 10

        return gain


    def compute_gain(self, origin_coord, reachable_points):
        gains = []
        # rds1 = self.ray_tracer.sensor_config['range']
        print("size ", self.ray_tracer.cost_map.shape)
        x_dist = (self.ray_tracer.cost_map.shape[0]/2)  * 0.05
        y_dist = (self.ray_tracer.cost_map.shape[1]/2)  * 0.05
        x_dist = abs(x_dist - abs(origin_coord[0]))
        y_dist = abs(y_dist - abs(origin_coord[1]))
        if x_dist < y_dist:
            rds = x_dist 
        else:
            rds = y_dist  # 10 is the radius of sample points
        # if rds1 < rds :
        #     rds = rds1

        for point in reachable_points:
            gain = self.compute_gain_at(point, radius=rds)
            gains.append((gain, point))
        return sorted(gains)
        



    def get_goals(self, robot_pose):
        x = robot_pose.position.x
        y = robot_pose.position.y
        radial_points = self.get_radial_points((x,y))
        goals = self.get_reachable_points((x,y), radial_points)
        goal_gains = self.compute_gain((x,y), goals)
        return goals,goal_gains



if __name__ == '__main__':
    import rospy
    rospy.init_node('goal_manager', anonymous=True)
    g = GoalSampler()
    pts = g.get_radial_points((100,100), 10, math.pi/4)
    print(pts)


