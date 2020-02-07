#!/usr/bin/env python
import rospy
import json
import math

# ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import Point

# Class
class waypoints_handler(object):
    def __init__(self):
        self.waypoint_list = {}
        self.waypoint_index = 0
    
        # Init ROS node
        rospy.init_node('waypoints_handler')
        self.freq = 10
        self.rate = rospy.Rate(self.freq)
    
        # Publishers
        self.waypoint_pub = rospy.Publisher('waypoint', Point, queue_size = 10)

        # Subscribers
        rospy.Subscriber('waypoint_series', String, self.seriesCB)
        rospy.Subscriber('waypoint/reached', Point, self.reachCB)
    
    def Start(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def pub_waypoint(self):
        if not self.waypoint_index in self.waypoint_list:
            return
        
        p = Point()
        p.x = float(self.waypoint_list[self.waypoint_index]["x"])
        p.y = float(self.waypoint_list[self.waypoint_index]["y"])
        p.z = float(self.waypoint_list[self.waypoint_index]["z"])
        self.waypoint_pub.publish(p)
            
    # ROS callback function
    def seriesCB(self, s):
        list_json = json.loads(s.data)
        self.waypoint_list = {}
        for point in list_json:
            self.waypoint_list[point["id"]] = point["point"]
        self.waypoint_index = 0
        self.pub_waypoint()
    
    def reachCB(self, p):
        current_p = self.waypoint_list[self.waypoint_index]
        dist = math.sqrt((current_p["x"]-p.x)**2 + (current_p["y"]-p.y)**2 + (current_p["z"]-p.z)**2)
        if dist < 0.2:
            self.waypoint_index += 1
        self.pub_waypoint()

if __name__ == '__main__': 
    hdl = waypoints_handler() 
    hdl.Start()
