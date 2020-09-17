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
from std_msgs.msg import Bool,Int32

#move_base
import tf
import math
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class LocalPlanner:

    def __init__(self):
        # Utilities
        self.m_manager = MapManager()
        self.r_monitor = RobotMonitor()
        self.g_sampler = GoalSampler()
        self.visualize = DarksideVisualizer()        

        # Planner variables
        self.gmap = None
        self.explored_map = None
        self.robot_pose = None
        self.goal_gains = []
        self.current_goal = (0,0)
        self.current_gain = 0
        self.last_goal = (10,10)
        self.last_gain = 0
        self.goal_samples = []
        self.previous_goals_list = []
        self.sampling_radius = 20
        self.radius_flag = True
        # self.alt_goals = []



    def update_map(self):

        self.m_manager.update_map()
        self.gmap = self.m_manager.get_map()
        self.g_sampler.set_ray_tracer_map(self.gmap,self.m_manager.get_map_origin())
        while self.gmap is None:
            rospy.loginfo("Waiting for map ")
            self.m_manager.update_map()
            self.gmap = self.m_manager.get_map()
            self.g_sampler.set_ray_tracer_map(self.gmap, self.m_manager.get_map_origin())

        return self.gmap


    def update_robot_pose(self):
        self.r_monitor.update_robot_pose()
        self.robot_pose = self.r_monitor.get_robot_pose()

    
    def update_goal(self):
        self.goal_samples, self.goal_gains = self.g_sampler.get_goals(self.robot_pose,radius=self.sampling_radius)
        if len(self.goal_gains) == 0:
            self.radius_flag=True
        while self.radius_flag :
            self.update_sampling_radius()
            self.goal_samples, self.goal_gains = self.g_sampler.get_goals(self.robot_pose,radius=self.sampling_radius)
            if  len(self.goal_gains) == 0:
                continue
            self.current_goal, self.current_goal = self.goal_gains[-1]
        #check if stuck at same pose
        print(self.current_goal)
        print(self.last_goal)
        x1,y1 = self.current_goal
        x2,y2 = self.last_goal

        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1)
        y2 = int(y2)

        if (x1==x2 and y1==y2):

            self.radius_flag=False
            self.update_sampling_radius()
            self.goal_samples, self.goal_gains = self.g_sampler.get_goals(self.robot_pose,radius=self.sampling_radius)
            if  len(self.goal_gains) > 0:
                self.current_goal, self.current_goal = self.goal_gains[-1]     
               
        self.last_goal = self.current_goal
        self.last_gain = self.current_gain
        # self.alt_goals.append(self.goal_gains[-2])
        self.previous_goals_list.append((self.current_goal,self.current_goal))
        self.update_sampling_radius()
        return self.current_goal


    def update_markers(self): 
        self.visualize.visualize_goal_samples(self.goal_samples, color=(0,1,0,1), lifetime=3)
        self.visualize.visualize_goal_samples([self.current_goal],color=(1,0,0,1), lifetime=6000)


    def update_sampling_radius(self,delta_radius=10):
        if len(self.goal_gains) == 0:
            self.sampling_radius -= delta_radius
            self.radius_flag = True
            rospy.loginfo("Sampling radius updated to %d", self.sampling_radius)
        else:
            if self.sampling_radius < 30:
                self.sampling_radius += 3
            self.radius_flag = False


class GlobalPlanner:

    def movebase_client(self,point,orn):
        rospy.loginfo("Goal received , X : %f , Y : %f",point[0],point[1])
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


    def foundCallback(self,msg):
        if msg.data== True:
            self.artifact_detected =True

    def scoreCallback(self,msg):
        if msg.data== True:
            self.artifact_scored = True


    def __init__(self):
        self.local_planner = LocalPlanner()
        self.way_points = []
        self.planning_step_num = 0
        self.local_plan_created = False
        self.local_plan_executed = False

        self.artifact_detected = False #add subscriber for artifact data
        self.artifact_scored = False    # add subscriber for artifact score

        self.found_subscriber = rospy.Subscriber("/X1/artifact_found_pub", Bool, self.foundCallback)
        self.score_subscriber = rospy.Subscriber("/subt/score", Int32, self.scoreCallback)


    def create_local_plan(self):
        self.local_plan_executed = False
        self.local_planner.update_map()
        self.local_planner.update_robot_pose()
        self.way_points.append(self.local_planner.update_goal())
        self.local_planner.update_markers()
        self.local_plan_created = True

    def execute_local_goal(self):
        pos = self.way_points[-1]
        yaw = self.local_planner.robot_pose.orientation.z
        self.movebase_client(pos, yaw)
        self.local_plan_created = False
        self.local_plan_executed = True

    
    def return_to_home(self):
        waypoints = self.way_points
        while not self.artifact_scored:
            if waypoints:
                x,y = waypoints.pop(-1)
                self.movebase_client((x,y), 0)