#!/usr/bin/env python

"""
The main module of the darkside planner , this will call the local planner to get the goal,
convert it into a PoseStamped message and send to the move_base module, and wait for,
execution completion from move_base.
Additionally it will evaluate mission objectives for the exploration
"""

import rospy
from visualizer import DarksideVisualizer
from goal_sampler import GoalSampler
from map_manager import MapManager, RobotMonitor
from raytrace_utils import RayTrace




class LocalPlanner:

    def __init__(self):
        self.m_manager = MapManager()
        self.r_monitor = RobotMonitor()
        self.g_sampler = GoalSampler()
        self.visualize = DarksideVisualizer()
        self.gmap = None
        self.get_map()
        self.current_goals = []
        self.past_goals = []

    def get_map(self):

        self.m_manager.update_map()
        self.gmap = m_manager.get_map()
        self.g_sampler.set_ray_tracer_map(self.gmap,self.m_manager.get_map_origin())
        while gmap is None:
            rospy.loginfo("Waiting for map ")
            self.m_manager.update_map()
            self.gmap = m_manager.get_map()
            self.g_sampler.set_ray_tracer_map(self.gmap,self.m_manager.get_map_origin())

        return self.gmap

    def get_goals(self):
        self.r_monitor.update_robot_pose()
        rpose = r_monitor.get_robot_pose()
        self.current_goals = self.g_sampler.get_goals(rpose)
        return self.current_goals


class GlobalPlanner:

    def __init__(self):

        # local planner stuff
        self.local_planner = None
        self.local_planning_done = False
        # list of functions to be called to check mission objectives like
        #  1.  Return to home
        #  2.  Increase/decrease search radius
        #  3.  
        self.mission_callbacks = []

        self.way_points = []
        self.planning_step_id = 0
        
    
    def get_local_goal()