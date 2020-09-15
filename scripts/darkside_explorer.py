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
        # if not planner.artifact_detected:    
        planner.create_local_plan()
        planner.execute_local_goal()
        if time.time() - time_start >= 7200:
            break
        # if planner.artifact_detected:
        #     planner.return_to_home()
        # if planner.artifact_scored:
        #     planner.artifact_detected = False
        #     planner.artifact_scored = False