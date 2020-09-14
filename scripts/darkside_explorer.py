#!/usr/bin/env python

import rospy
import time
from global_planning_test import GlobalPlanner


if __name__ == '__main__':
    rospy.init_node('darkside_explorer', anonymous=True)
    planner = GlobalPlanner()
    mission_completed = False
    time_start = time.time()
    while not rospy.is_shutdown() or not mission_completed:
        
        planner.create_local_plan()
        planner.execute_local_goal()
        if time.time() - time_start >= 3600:
            break