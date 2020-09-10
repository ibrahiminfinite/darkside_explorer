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
# from geometry_msgs import Pose, Point, PoseStamped


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

    def get_radial_points(self, origin, radius=4, step_size=math.pi/12):
        
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

    

    def compute_sample_gain(self, reachable_points):
        # TODO : compute gain
        pass


    def get_goals(self, robot_pose):
        x = robot_pose.position.x
        y = robot_pose.position.y
        radial_points = self.get_radial_points((x,y))
        goals = self.get_reachable_points((x,y), radial_points)
        return goals



if __name__ == '__main__':
    import rospy
    rospy.init_node('goal_manager', anonymous=True)
    g = GoalSampler()
    pts = g.get_radial_points((100,100), 10, math.pi/4)
    print(pts)


