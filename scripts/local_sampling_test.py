#!/usr/bin/env python


import rospy
from visualizer import DarksideVisualizer
from goal_sampler import GoalSampler
from map_manager import MapManager, RobotMonitor
from raytrace_utils import RayTrace



if __name__ == '__main__':
    rospy.init_node('local_manager', anonymous=True)
    # create clients
    m_manager = MapManager()
    r_monitor = RobotMonitor()
    g_sampler = GoalSampler()
    ry_tracer = RayTrace()
    visualize = DarksideVisualizer()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #Get map
        m_manager.update_map()
        m_manager.get_map()
        gmap = m_manager.get_map()
        r_monitor.update_robot_pose()
        rpose = r_monitor.get_robot_pose()

        print(gmap)
        print("\n\n\n")
        print(rpose)
        r.sleep()