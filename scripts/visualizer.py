#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class DarksideVisualizer:

    def __init__(self):
        self.goal_position = rospy.Publisher('/darkside_explorer/local_goal', Marker, queue_size=1)
        self.marker_id = 0

    def make_marker(self, marker_points, scale_=[0.3,0.3,0.3], color_=(0,0,1,1), lifetime_= 1):

        marker_ = Marker()
        marker_.id = self.marker_id
        
        self.marker_id +=1
        marker_.header.frame_id = "/X1/map"

        # marker_.header.stamp = rospy.Time.now()
        marker_.type = marker_.POINTS
        marker_.action = marker_.ADD

        marker_.points = marker_points

        marker_.lifetime = rospy.Duration.from_sec(lifetime_)

        marker_.scale.x = scale_[0]
        marker_.scale.y = scale_[1]
        marker_.scale.z = scale_[2]
        marker_.color.a = 0.5
        red_, green_, blue_, a_ = color_
        marker_.color.r = red_
        marker_.color.g = green_
        marker_.color.b = blue_
        marker_.color.a = a_
        return marker_

    def make_points(self, points):
        marker_points = []
        for point in points :
            p = Point()
            x,y = point
            p.x, p.y, p.z = x, y, 0
            marker_points.append(p)
        return marker_points

    def publish_markers(self, markers):
        self.goal_position.publish(markers)


    def visualize_goal_samples(self, goal_points):
        ros_points = self.make_points(goal_points)
        self.publish_markers(self.make_marker(ros_points, lifetime_= 300, color_=(0,1,0,1)))
        rospy.loginfo("Published goal visualizer")


"""
radial point viz 
"""


# def get_points(rpose):
#     points = g.get_radial_points((rpose.position.x,rpose.position.y), radius=6, step_size=math.pi/6)
#     print("POINTS : ", len(points))
#     point_array = viz.make_points(points)
#     print("POSES : ", len(point_array))
#     marker = viz.make_marker(point_array)
#     return marker

# def poseCallback(msg):
#     markers = get_points(msg)
#     print("MARKERS :", len(markers.points))
#     viz.publish_markers(markers)
#     rospy.loginfo("Displaying marker")

# if __name__ == '__main__':
#     rospy.init_node('viz_manager', anonymous=True)
#     from goal_sampler import GoalSampler
#     g = GoalSampler()
#     viz = DarksideVisualizer()
#     rospy.Subscriber("/robot_pose", Pose, poseCallback)
#     rospy.spin()


"""
ray cast viz 
"""



# def get_points(rpose):
#     x, y = int(rpose.position.x/0.05),int(rpose.position.y/0.05)
#     points = r.cast_ray((x, y),(500,500))[2] 
#     points = [(x*0.05, y*0.05) for x,y in points]
#     print("POINTS : ", len(points))
#     point_array = viz.make_points(points)
#     print("POSES : ", len(point_array))
#     marker = viz.make_marker(point_array,scale_=[0.05,0.05,0.05], color_=(0,0,1,1))
#     return marker

# def poseCallback(msg):
#     markers = get_points(msg)
#     print("MARKERS :", len(markers.points))
#     viz.publish_markers(markers)
#     rospy.loginfo("Displaying marker")

# if __name__ == '__main__':
#     rospy.init_node('viz_manager', anonymous=True)
#     from raytrace_utils import RayTrace
#     r = RayTrace()
#     viz = DarksideVisualizer()
#     rospy.Subscriber("/robot_pose", Pose, poseCallback)
#     rospy.spin()