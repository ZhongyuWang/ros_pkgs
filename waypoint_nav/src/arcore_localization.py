#!/usr/bin/env python
import rospy
import math
import tf

# ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

# Class
class arcore_localization(object):
    def __init__(self):

        # Init ROS node
        rospy.init_node('arcore_localization')
        self.freq = 10
        self.rate = rospy.Rate(self.freq)

        # Publishers
        self.robot_odom_pub = rospy.Publisher('arcore/robot_odom', Odometry, queue_size = 10)

        # Subscribers
        rospy.Subscriber('arcore/cam_pose', Pose, self.poseCB)

    def Start(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    # ROS Callback functions
    def poseCB(self, p):
        pose = p
        euler = tf.transformations.euler_from_quaternion((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        
        # Transformation needs to be applied if camera center is not align with robot center
        odom = Odometry()
        odom.header.frame_id = "arcore_map"
        odom.header.stamp = rospy.Time.now()
        odom.child_frame_id = "arcore_base"
        odom.pose.pose.position = p.position
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(euler[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(euler[2] / 2.0)
        self.robot_odom_pub.publish(odom)
    
if __name__ == '__main__': 
    arcore = arcore_localization() 
    arcore.Start()

