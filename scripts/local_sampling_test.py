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
    visualize = DarksideVisualizer()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
            
        #Get map
        gmap = m_manager.get_map()
        g_sampler.set_ray_tracer_map(gmap)
        while gmap is None:
            rospy.loginfo("Waiting for map ")
            gmap = m_manager.get_map()
            g_sampler.set_ray_tracer_map(gmap, m_manager.get_map_origin())


        r_monitor.update_robot_pose()
        rpose = r_monitor.get_robot_pose()

        goals = g_sampler.get_goals(rpose)
        visualize.visualize_goal_samples(goals)
        r.sleep()