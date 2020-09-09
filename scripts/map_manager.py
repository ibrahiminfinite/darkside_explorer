#!/usr/bin/env python
"""
Map Manager that subscribes to and maintains the map that the explorer will use
"""

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid


class RobotMonitor:

    def __init__(self, robot_pose_topics="/robot_pose"):
        self.pose_subscriber = rospy.Subscriber("/robot_pose", Pose, self.poseCallback)
        self.robot_pose = None
        self.update_pose = True

    def poseCallback(self,pose_msg):
        if self.update_pose:
            self.robot_pose = pose_msg
            self.update_pose = False
        
    def get_robot_pose(self):
        return self.robot_pose

    def update_robot_pose(self):
        self.update_pose = True
        rospy.sleep(0.5)


class MapManager:

    def mapCallback(self,map_msg):
                       
        if self.update:
            #map_info
            self.map_raw = map_msg.data 
            
            self.map_height_in_cells = map_msg.info.height
            self.map_width_in_cells  = map_msg.info.width
            self.map_resolution = map_msg.info.resolution
            self.origin_x = map_msg.info.origin.position.x
            self.origin_y = map_msg.info.origin.position.y

            rospy.loginfo("Height : %d ,  Width : %d ", self.map_height_in_cells, self.map_width_in_cells)
            rospy.loginfo("OriginX : %d ,  OriginY : %d ", self.origin_x, self.origin_y)

            self.map_last_update_time = rospy.Time.now()
            self.raw_to_numpy(self.map_raw) #converts raw map array to numpy
            self.update = False
        else:
            rospy.loginfo("New map update received, but not updated")


    def __init__(self):

        self.update = True #/X1/move_base/global_costmap/costmap
        self.map_subscriber = rospy.Subscriber("/X1/move_base/global_costmap/costmap", OccupancyGrid, self.mapCallback)
        self.numpy_map = None
        self.map_raw = None
        self.map_last_update_time = rospy.Time.now()


    def raw_to_numpy(self, raw_map):   
        numpy_map = list(list())
        for row in range(self.map_height_in_cells)[::-1]:
            row_i = []
            for column in range(self.map_width_in_cells):
                row_i.append(self.map_raw[(row*self.map_width_in_cells) + column ])
            numpy_map.append(row_i)
        self.numpy_map = np.asarray(numpy_map)
    
    def get_map(self):
        empty_map =False
        try:
            if self.numpy_map.size == 0:
                rospy.logwarn("Map data is 'None'")
                empty_map = True
        except:
            if self.numpy_map == None :
                rospy.logwarn("Map data is 'None'")
                empty_map = True
 
        if not empty_map:
            return self.numpy_map
        else : 
            return None

    def update_map(self):
        if not self.update:
            self.update = True 

            
if __name__ == '__main__':
    rospy.init_node('map_manager', anonymous=True)
    m = MapManager()
    rospy.spin()