#!/usr/bin/env python

"""
The main module of the darkside planner , this will call the local planner to get the goal,
convert it into a PoseStamped message and send to the move_base module, and wait for,
execution completion from move_base.
Additionally it will evaluate mission objectives for the exploration
"""

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
        