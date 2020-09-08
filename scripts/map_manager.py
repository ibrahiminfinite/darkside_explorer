#!/usr/bin/env python
"""
Map Manager that subscribes to and maintains the map that the explorer will use
"""

import rospy
from nav_msgs.msg import OccupancyGrid


class MapManager:

    def mapCallback(self,map_msg):
                       
        if self.update_map:
            #map_info
            self.map_raw = map_msg.data 
            self.map_height_in_cells = map_msg.info.height
            self.map_width_in_cells  = map_msg.info.width
            self.map_resolution = map_msg.info.resolution
            self.origin_x = map_msg.info.origin.position.x
            self.origin_y = map_msg.info.origin.position.y

            rospy.loginfo("Height : %d ,  Width : %d ", self.map_height_in_cells, self.map_width_in_cells)
            rospy.loginfo("OriginX : %d ,  OriginY : %d ", self.origin_x, self.origin_y)
            # map data 
            # self.map_last_update_time = rospy.Time.now()
            # self.raw_to_grid() #converts raw map array to grid
            # self.update_map = False
        else:
            rospy.loginfo("New map update received, but not updated")


    def __init__(self):

        self.update_map = True #/X1/move_base/global_costmap/costmap
        self.map_subscriber = rospy.Subscriber("/X1/move_base/global_costmap/costmap", OccupancyGrid, self.mapCallback)
        self.map = None
        self.map_raw = None
        self.map_last_update_time = rospy.Time.now()
        
    
    def get_map(self):
        if self.map == None:
            rospy.logwarn("Map data is 'None'")
        else:
            return self.map

    def update_map(self):
        if not self.update_map:
            self.update_map = True 

            
if __name__ == '__main__':
    rospy.init_node('map_manager', anonymous=True)
    m = MapManager()
    rospy.spin()