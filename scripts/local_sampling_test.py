#!/usr/bin/env python


import rospy
from visualizer import DarksideVisualizer
from goal_sampler import GoalSampler
from map_manager import MapManager, RobotMonitor
from raytrace_utils import RayTrace

import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




def movebase_client(point,orn):

    client = actionlib.SimpleActionClient('X1/move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "X1/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point[0]
    goal.target_pose.pose.position.y = point[1]
    x , y, z, w = tf.transformations.quaternion_from_euler(0, 0, orn)
    goal.target_pose.pose.orientation.x = x
    goal.target_pose.pose.orientation.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    rospy.init_node('local_manager', anonymous=True)
    # create clients
    m_manager = MapManager()
    r_monitor = RobotMonitor()
    g_sampler = GoalSampler()
    visualize = DarksideVisualizer()
    g_list = []
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
            
        #Get map
        m_manager.update_map()
        gmap = m_manager.get_map()
        g_sampler.set_ray_tracer_map(gmap,m_manager.get_map_origin())
        while gmap is None:
            rospy.loginfo("Waiting for map ")
            m_manager.update_map()
            gmap = m_manager.get_map()
            g_sampler.set_ray_tracer_map(gmap, m_manager.get_map_origin())


        r_monitor.update_robot_pose()
        rpose = r_monitor.get_robot_pose()

        goals, goal_gains = g_sampler.get_goals(rpose)
        print(goal_gains[-1][1])
        # print(goals[-1])
        g_list.append(goal_gains[-1][1])
        visualize.visualize_goal_samples(g_list)
        
        movebase_client(goal_gains[-1][1], rpose.orientation.z)
        r.sleep()