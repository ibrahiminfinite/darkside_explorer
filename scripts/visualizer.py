#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class DarksideVisualizer:

    def __init__(self):
        self.goal_position = rospy.Publisher('/darkside_explorer/local_goal', Marker, queue_size=1)

    def make_marker(self, marker_pose, scale_=[0.3,0.3,0.3], color_=(0,0,1,1), lifetime_=300):
        marker_array_ = []
        marker_ = Marker()
        marker_.header.frame_id = "/X1/map"
        # marker_.header.stamp = rospy.Time.now()
        marker_.type = marker_.SPHERE
        marker_.action = marker_.ADD

        marker_.pose = marker_pose

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

    def make_pose(self, robot_pose, point):
        pose_t = robot_pose
        pose_t.position.x = point[0]
        pose_t.position.y = point[1]

        return pose_t

    



def poseCallback(msg):
    viz.show_marker(msg)
    rospy.loginfo("Displaying marker")

if __name__ == '__main__':
    rospy.init_node('viz_manager', anonymous=True)
    from goal_sampler import GoalSampler
    g = GoalSampler()
    viz = DarksideVisualizer()
    rospy.Subscriber("/robot_pose", Pose, poseCallback)
    rospy.spin()