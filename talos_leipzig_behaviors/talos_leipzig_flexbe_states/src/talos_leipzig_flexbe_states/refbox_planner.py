#!/usr/bin/env python

import rospy
import math
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from atwork_commander_msgs.msg import Object

'''
Created: 27/06/2024

@author: Altzi Tsanko
email: ace.tsan21@gmail.com
'''

class RefBoxPlannerState(EventState):
    '''
    Robocup@Work referee box state. Takes a parsed task dictionary as input and iteratively extracts 
    an arena_start_state goal and then an arena_end_state goal by matching object lists to find target goal. 
    For each goal, it communicates the workstation location information as a Pose-goal to the navigation 
    states and the provided object lists to the object detection state to prepare for a manipulation goal.

    Built to satisfy the Basic Manipulation Task (BMT) needs.

    ># task              dict    Python dictionary containing the parsed task.
    ># sa_poses          dict    Python dictionary containing the mapped workstations to coordinates and object IDs to poses.

    #> object_goal       list    Contains current goal objects to be ideally detected by the object_detection state 
                                 and eventually manipulated. 
    #> waypoint          Pose2D  Contains the pose of the current workstation goal for navigation purposes.
    #> workstation_name  str     Contains the name of the current workstation goal for manipulation purposes.
    #> task              dict    Updated task dictionary with the current processed goal removed.

    <= continue
    <= failed
    <= test_exhausted
    '''

    def __init__(self):
        super(RefBoxPlannerState, self).__init__(
            input_keys=['task', 'sa_poses', 'flag_start'], 
            output_keys=['waypoint', 'task', 'flag_start'], 
            outcomes=['continue', 'failed', 'test_exhausted']
        )
    
        self.limit = 3  # Specifies the maximum number of items to carry at the same time
        self.odom_topic = '/odom'
        self.odom_sub = ProxySubscriberCached({self.odom_topic: Odometry})
        self.flag = None
    
    def on_enter(self, userdata):
        """ Initialize the state when it is entered. """
        if userdata.flag_start:
            self.task_dict = userdata.task  # Read task the first time state is initialized, then updated internally
            self.sa_poses_dict = userdata.sa_poses
            
            self.take_obj = self.task_dict[0]
            self.leave_obj = self.task_dict[1]
            self.remaining_src = self.take_obj.copy()
            self.remaining_dest = self.leave_obj.copy()
            
            for key in self.remaining_src:
                if key not in self.remaining_dest:
                    self.remaining_dest[key] = []
                    
            self.carry = []

    def execute(self, userdata):
        """ Execute the state logic. """
        if not self.odom_sub.has_msg(self.odom_topic):
            return 'failed'

        if self.odom_sub.has_msg(self.odom_topic):
            self.data = self.odom_sub.get_last_msg(self.odom_topic)
            self.odom_sub.remove_last_msg(self.odom_topic)

        if not self.remaining_src and not self.remaining_dest:
            return 'test_exhausted'

        elif len(self.carry) < self.limit and self.remaining_src:
            # Find the nearest service area
            nsa = self.nearest_service_area(self.remaining_src)
            service_area_goal = self.sa_poses_dict[nsa]['ws_pose']
            userdata.waypoint = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose2D', service_area_goal)

            if set(self.carry) & set(self.remaining_dest[nsa]):
                self.carry, self.remaining_dest[nsa] = self.remove_common(self.carry, self.remaining_dest[nsa])

            self.remaining_src = {i: j for i, j in self.remaining_src.items() if j}
            self.remaining_dest = {i: j for i, j in self.remaining_dest.items() if j}

            self.carry.extend(self.remaining_src[nsa][:self.limit - len(self.carry)])
            del self.remaining_src[nsa][:len(self.carry)]

            self.remaining_src = {i: j for i, j in self.remaining_src.items() if j}
            self.remaining_dest = {i: j for i, j in self.remaining_dest.items() if j}
            
            return 'continue'

        elif len(self.carry) == self.limit or not self.remaining_src:
            for p in self.remaining_dest.keys():
                if set(self.carry) & set(self.remaining_dest[p]):
                    service_area_goal = self.sa_poses_dict[p]['ws_pose']
                    userdata.waypoint = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose2D', service_area_goal)

                    self.carry, self.remaining_dest[p] = self.remove_common(self.carry, self.remaining_dest[p])

                    self.remaining_src = {i: j for i, j in self.remaining_src.items() if j}
                    self.remaining_dest = {i: j for i, j in self.remaining_dest.items() if j}

                    if self.remaining_src:
                        self.carry.extend(self.remaining_src[p][:len(self.carry)])
                        del self.remaining_src[p][:len(self.carry)]

                    self.remaining_src = {i: j for i, j in self.remaining_src.items() if j}
                    self.remaining_dest = {i: j for i, j in self.remaining_dest.items() if j}
                    
                    return 'continue'   

        return 'failed'

    def nearest_service_area(self, ws):
        """ Find the nearest service area based on the current robot position. """
        distances = {}
        for p in ws:
            distance = math.sqrt(
                (self.data.pose.pose.position.x - self.sa_poses_dict[p]['ws_pose']['x'])**2 + 
                (self.data.pose.pose.position.y - self.sa_poses_dict[p]['ws_pose']['y'])**2
            )
            distances[p] = distance
        return min(distances, key=distances.get)

    def remove_common(self, a, b):
        """ Remove common elements between two lists. """
        a_copy = a[:]
        for item in a_copy:
            if item in b:
                a.remove(item)
                b.remove(item)
        return a, b

    def on_exit(self, userdata):
        """ Clean up when exiting the state. """
        pass
