#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from laser_line_extraction.msg import LineSegmentList
from laser_line_extraction.msg import LineSegment
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from tf.transformations import euler_from_quaternion

'''
Created: 09/07/2024

@author: Altzi Tsanko
email: ace.tsan21@gmail.com
'''


class pose_approach(EventState):
    '''
    State for center approach to a service area in the simulation environment. Finds the center of the line segment to
    calculate the robot's distance and publish cmd_vel messages in order to achieve the goal.

    ># height_sa    int         The height of the service area table
    
    <= success
    <= failed
    '''

    def __init__(self):
        super(pose_approach, self).__init__(outcomes=['success', 'failed'],
                                            input_keys=['height_sa'])

        self._sub_topic = '/line_segments'
        self._pub_topic = '/cmd_vel'
        self.line_sub = ProxySubscriberCached({self._sub_topic: LineSegmentList})
        self.line_sub.set_callback(self._sub_topic, self.line_callback)
        self.line_sub.make_persistant(self._sub_topic)
        self.cmd_pub = ProxyPublisher({self._pub_topic: Twist})
        self.rate = rospy.Rate(10)

    def get_distance_angle_and_center(self, line_segment):
        start = line_segment.start
        end = line_segment.end

        # Calculate distance and center
        center_x = (start[0] + end[0]) / 2.0
        center_y = (start[1] + end[1]) / 2.0
        distance = math.sqrt(center_x**2 + center_y**2)

        # Calculate angle
        angle = math.atan2(end[1] - start[1], end[0] - start[0])

        return distance, angle, (center_x, center_y)
    
    def move_towards_center(self, angle, distance):
        self.flag = False
        cmd_vel = Twist()
        stopping_distance = 0.34  # Desired stopping distance from the line center
        linear_speed = 0.05  # Linear speed of the robot

        if distance > stopping_distance:
            # Move towards the center
            cmd_vel.linear.x = linear_speed
            cmd_vel.linear.y = 0
            cmd_vel.angular.z = self.calculate_angular_velocity(angle)
            rospy.loginfo("Closest line center at: %s", distance)
        else:
            # Stop the robot
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.angular.z = 0
            rospy.loginfo("Robot has reached the desired distance from the line center. Stopping.")
            self.flag = True

        self.cmd_pub.publish(cmd_vel)

    def calculate_angular_velocity(self, target_angle):
        # Get current orientation from the robot's pose
        (roll, pitch, yaw) = euler_from_quaternion([0, 0, 0, 1])  # No rotation for now

        # Calculate angle difference
        angle_difference = target_angle - yaw

        # Adjust angular velocity proportional to the angle difference
        proportional_gain = 0.025
        angular_velocity = proportional_gain * angle_difference

        return angular_velocity

    def line_callback(self, data):
        self.lines = data.line_segments

    def on_start(self):
        Logger.loginfo("approach")

    def on_enter(self, userdata):
        Logger.loginfo("approach STARTED!")

    def execute(self, userdata):
        if not self.cmd_pub:
            return 'failed'
        
        if not self.lines:
            rospy.loginfo("No line segments detected.")
            return 
        # Find the closest line segment
        closest_distance = float('inf')
        closest_angle = None
        closest_center = None
        for segment in self.lines:
            distance, angle, center = self.get_distance_angle_and_center(segment)
            if distance < closest_distance:
                closest_distance = distance
                closest_angle = angle
                closest_center = center

        if closest_center:
            #rospy.loginfo("Closest line center at: %s", closest_center)
            self.move_towards_center(closest_angle, closest_distance)
        
        if userdata.height_sa < 5:
            return 'success'

        if self.flag == True:
            return 'success'
                 

    def on_exit(self, userdata):
        #self.line_sub.unsubscribe_topic(self._sub_topic)
        Logger.loginfo("pose_approach ENDED!")