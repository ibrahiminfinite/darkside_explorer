#!/usr/bin/env python

import rospy
from global_planning_test import GlobalPlanner


if __name__ == '__main__':
    rospy.init_node('darkside_explorer', anonymous=True)
    planner = GlobalPlanner()
    mission_completed = False
    while not rospy.is_shutdown() or not mission_completed:
        
        planner.create_local_plan()
        planner.execute_local_goal()
        