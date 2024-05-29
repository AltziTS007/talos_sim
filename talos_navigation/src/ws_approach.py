#!/usr/bin/env python

import rospy
import math
from laser_line_extraction.msg import LineSegmentList
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import euler_from_quaternion

def get_distance_angle_and_center(line_segment):
    start = line_segment.start
    end = line_segment.end

    # Calculate distance and center
    center_x = (start[0] + end[0]) / 2.0
    center_y = (start[1] + end[1]) / 2.0
    distance = math.sqrt(center_x**2 + center_y**2)

    # Calculate angle
    angle = math.atan2(end[1] - start[1], end[0] - start[0])

    return distance, angle, (center_x, center_y)

def callback(data):
    if not data.line_segments:
        rospy.loginfo("No line segments detected.")
        return

    # Find the closest line segment
    closest_distance = float('inf')
    closest_angle = None
    closest_center = None
    for segment in data.line_segments:
        distance, angle, center = get_distance_angle_and_center(segment)
        if distance < closest_distance:
            closest_distance = distance
            closest_angle = angle
            closest_center = center

    if closest_center:
        rospy.loginfo("Closest line center at: %s", closest_center)
        move_towards_center(closest_center, closest_angle, closest_distance)

def move_towards_center(center, angle, distance):
    cmd_vel = Twist()
    stopping_distance = 0.02  # Desired stopping distance from the line center
    linear_speed = 0.1  # Linear speed of the robot

    if distance > stopping_distance:
        # Move towards the center
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = calculate_angular_velocity(angle)
    else:
        # Stop the robot
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        rospy.loginfo("Robot has reached the desired distance from the line center. Stopping.")

    cmd_vel_pub.publish(cmd_vel)

def calculate_angular_velocity(target_angle):
    # Get current orientation from the robot's pose
    (roll, pitch, yaw) = euler_from_quaternion([0, 0, 0, 1])  # No rotation for now

    # Calculate angle difference
    angle_difference = target_angle - yaw

    # Adjust angular velocity proportional to the angle difference
    proportional_gain = 0.1
    angular_velocity = proportional_gain * angle_difference

    return angular_velocity

def listener():
    rospy.init_node('line_follower', anonymous=True)
    rospy.Subscriber('/line_segments', LineSegmentList, callback)
    rospy.spin()

if __name__ == '__main__':
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    listener()
