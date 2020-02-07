#!/usr/bin/env python
import rospy
import math
import tf

# ROS messages
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry

# Class
class mobile_waypoint_control(object):
    def __init__(self):
        # waypoint control state
        self.STOP = 0
        self.TURNING = 1
        self.FORWARDING = 2

        # State variables
        self.node_state = False
        self.robot_state = self.STOP
    
        # Control parameters
        self.FWD_K = 0.3
        self.ANG_K = 0.5
        self.MAX_LIN_VEL = 0.5
        self.MAX_ANG_VEL = 1.0
        self.FWD_THRES = 0.1
        self.TURN_THRES = 0.2
    
        # Variables
        self.goal_set = False
        self.pose_get = False
        self.vel = Twist()
        self.waypoint = Point()
        self.robot_pose = Pose()
    
        # Init ROS node
        rospy.init_node('waypoint_control')
        self.freq = 10
        self.rate = rospy.Rate(self.freq)

        # Publishers
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.reached_pub = rospy.Publisher('waypoint/reached', Point, queue_size = 10)

        # Subscribers
        rospy.Subscriber('waypoint/node_state', Bool, self.stateCB)
        rospy.Subscriber('robot_pose', Pose, self.poseCB)
        rospy.Subscriber('robot_odom', Odometry, self.odomCB)
        rospy.Subscriber('waypoint', Point, self.waypointCB)
        rospy.Subscriber('waypoint/control_parameters', String, self.ctrlParaCB)
        rospy.Subscriber('waypoint/velocity_parameters', String, self.velParaCB)
        rospy.Subscriber('waypoint/threshold_parameters', String, self.thresParaCB)
    
    def Start(self):
        while not rospy.is_shutdown():
            # Wait until waypoint and robot is set
            if not self.goal_set and not self.pose_get:
                self.rate.sleep()
                continue

            # When the node is disabled, stop the robot if it is running
            if not self.node_state:
                if not self.robot_state == self.STOP:
                    self.StopRobot()   
                self.rate.sleep()
                continue
            
            # Calculate distance and angle to the waypoint
            point = self.waypoint
            pose = self.robot_pose
            distance = math.sqrt((pose.position.x - point.x)**2 + (pose.position.y - point.y)**2)
            euler = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            angle = math.atan2(point.y - pose.position.y, point.x - pose.position.x) - euler[2]
            angle = self.fitInRad(angle)
                
            if self.robot_state == self.TURNING:
                self.vel.linear.x = 0
                self.vel.angular.z = self.ANG_K * angle

                # Check whether turning process is finished
                finished_turning = True
                if math.fabs(angle) > self.TURN_THRES: 
                    finished_turning = False       
                if finished_turning:
                    self.vel.angular.z = 0
                    self.robot_state = self.FORWARDING

            # When the robot is moving forwarding    
            elif self.robot_state == self.FORWARDING:
                self.vel.linear.x = self.FWD_K * distance
                self.vel.angular.z = self.ANG_K * angle

                # Check whether the waypoint is reached
                finished_forwarding = True
                if math.fabs(distance) > self.FWD_THRES:
                    finished_forwarding = False
                if finished_forwarding:
                    self.reached_pub.publish(self.waypoint)
                    self.goal_set = False 
                    self.StopRobot()

                # If the orientation off too much, enter TURNING mode
                turning_needed = False
                if math.fabs(angle) > math.pi/4:  
                    turning_needed = True
                if turning_needed:
                    self.robot_state = self.TURNING

            else:
                # If waypoint/state and waypoint is set, robot should run
                if self.goal_set:
                    self.robot_state = self.TURNING

            # Fit the velocity into the limited range    
            self.vel.linear.x = self.LimitRange(self.vel.linear.x, self.MAX_LIN_VEL)
            self.vel.angular.z = self.LimitRange(self.vel.angular.z, self.MAX_ANG_VEL)
            # Publish cmd_vel
            self.vel_pub.publish(self.vel)     
            self.rate.sleep()
  
    # Control functions
    def StopRobot(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.robot_state = self.STOP
        self.vel_pub.publish(self.vel)
    
    def fitInRad(self, r):
        while r > math.pi:
            r = r - 2 * math.pi
        while r < -math.pi:
            r = r + 2 * math.pi
        return r
    
    def LimitRange(self, v, l):
        if v > 0:
            return min(v, math.fabs(l))
        else:
            return max(v, -math.fabs(l))
  
    # ROS callback function
    def stateCB(self, b):
        self.node_state = b.data
    
    def poseCB(self, p):
        self.robot_pose = p
        self.pose_get = True
    
    def odomCB(self, o):
        self.robot_pose = o.pose.pose
        self.pose_get = True
  
    def waypointCB(self, p):
        self.waypoint = p
        self.goal_set = False
        self.robot_state = self.TURNING
        if self.pose_get:
            self.node_state = True
    
    def ctrlParaCB(self, p):
        if len(p.data) <= 0:
            return
        parts = p.data.split(',')
        if len(parts) != 2:
            return
        try:
            self.FWD_K = float(parts[0])
            self.ANG_K = float(parts[1])
        except:
            return
        
    def velParaCB(self, p):
        if len(p.data) <= 0:
            return
        parts = p.data.split(',')
        if len(parts) != 2:
            return
        try:
            self.MAX_LIN_VEL = float(parts[0])
            self.MAX_ANG_VEL = float(parts[1])
        except:
            return

    def thresParaCB(self, p):
        if len(p.data) <= 0:
            return
        parts = p.data.split(',')
        if len(parts) != 2:
            return
        try:
            self.FWD_THRES = float(parts[0])
            self.TURN_THRES = float(parts[1])
        except:
            return

if __name__ == '__main__': 
    ctrl = mobile_waypoint_control() 
    ctrl.Start()

