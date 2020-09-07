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
from geometry_msgs import Pose, Point, PoseStamped


class Goal:
    
    def __init__(self, gpose, gain):
        self.pose = gpose
        self.gain = gain


class GoalSampler:

    def __init__(self, angular_step_size=0.01, radial_length=1, radial_relaxation=0.5):
        self.angular_step_size = angular_step_size # determines the number of samples
        self.radial_length = radial_length # in meters
        self.radial_relaxation = radial_relaxation # distance to increase or decrease if no goal found in meters
        self.robot_origin = None

    def get_radial_points(self, x, y, radius=self.radial_length, step_size=self.angular_step_size):
        
        ang = 0 
        radial_points = []
        x = x
        y = y
        while ang <= math.pi:                        
            radial_points.append((x + radius*math.cos(ang),y + radius*math.sin(ang)))
            ang += step_size
        
        return radial_points

    def get_reachable_points(self, radial_points):

        reachable_points = []
        for point in radial_points:
            if self.is_reachable_in_straight_line(point):
                reachable_points.append(point)

        return reachable_points

    

    def compute_sample_gain(self, reachable_points):
        # TODO : compute gain
