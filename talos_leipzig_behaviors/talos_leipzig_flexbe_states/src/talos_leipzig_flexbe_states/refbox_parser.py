#!/usr/bin/env python

import rospy
import os
import yaml
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from atwork_commander_msgs.msg import ObjectTask, Object

'''
Created: 27/06/2024

@author: Altzi Tsanko
email: ace.tsan21@gmail.com
'''

class RefBoxParserState(EventState):
    ''' 
    Robocup@Work referee box state. Parses useful info from generated task messages, maps generated object IDs
    to their respective object names and utilizes semantic information to map workstation names to a 2D pose.
    Outputs useful information to a Python dictionary to be used by the planner.  
    
    #> task dict  python dictionary containing the parsed task with mapped workstations to coordinates and object IDs to poses.
    #> sa_poses dict  semantic map data containing workstation poses.
    #> flag_start bool  flag indicating the start of the task.

    <= continue
    <= error_parsing
    ''' 

    def __init__(self):
        super(RefBoxParserState, self).__init__(outcomes=['continue', 'error_parsing'], output_keys=['task', 'sa_poses', 'flag_start'])
        self.objtask_topic = 'atwork_commander/object_task'
        # Set the relative path to the semantic map
        script_dir = os.path.dirname(__file__)
        self.semantic_map_path = os.path.join(script_dir, 'semantic_map', 'semantic_map.yaml')

        # Subscribe to object_task topic
        self.objtask_sub = ProxySubscriberCached({self.objtask_topic: ObjectTask})

    def on_start(self):
        """ Load semantic map when the state machine starts. """
        try:
            with open(self.semantic_map_path, 'r') as stream:
                semantic_map = yaml.safe_load(stream)
                self.semantic_map = {k: v for d in semantic_map for k, v in d.items()}  # Convert list of dictionaries to dictionary
        except yaml.YAMLError as exc:
            Logger.logerror(f"Error loading semantic map: {exc}")
            self.semantic_map = {}

    def on_enter(self, userdata):
        """ Wait for a task message to be published. """
        while not self.objtask_sub.has_msg(self.objtask_topic):
            Logger.logwarn('Refbox atwork_commander/object_task topic is not being published.')
            rospy.sleep(0.5)

        Logger.loginfo('Received task message.')
        
        # Initialize dictionaries for objects to take and leave
        self.take_obj = {ws: [] for ws in self.semantic_map.keys()}
        self.leave_obj = {ws: [] for ws in self.semantic_map.keys()}

        # Dictionary for all objects we have
        object_msg = Object()
        self.obj_dict = {
            object_msg.F20_20_B: 'F20_20_B',
            object_msg.F20_20_G: 'F20_20_G',
            object_msg.AXIS2: 'Axis2',
            object_msg.BEARING2: 'Bearing2',
            object_msg.S40_40_G: 'S40_40_G',
            object_msg.S40_40_B: 'S40_40_B',
            object_msg.M20: 'M20',
            object_msg.M30: 'M30',
            object_msg.M20_100: 'M20_100',
            object_msg.HOUSING: 'Housing',
            object_msg.MOTOR2: 'Motor2',
            object_msg.SPACER: 'Spacer',
            object_msg.SCREWDRIVER: 'Screwdriver',
            object_msg.WRENCH: 'Wrench',
            object_msg.DRILL: 'Drill',
            object_msg.ALLENKEY: 'AllenKey',
        }

    def execute(self, userdata):
        """ Execute the state logic. """
        if not self.objtask_sub.has_msg(self.objtask_topic):
            return 'error_parsing'

        # Retrieve and process the last data from Object_Task topic
        task_msg = self.objtask_sub.get_last_msg(self.objtask_topic)
        self.objtask_sub.remove_last_msg(self.objtask_topic)
        self.task = task_msg.subtasks

        Logger.loginfo(f'Objects: {len(self.task)}')

        # Populate dictionaries from the topic
        for src in self.task:
            self.take_obj[src.source].append(src.object.object)
            self.leave_obj[src.destination].append(src.object.object)

        # Remove empty workstation goals
        self.take_obj = {ws: objs for ws, objs in self.take_obj.items() if objs}
        self.leave_obj = {ws: objs for ws, objs in self.leave_obj.items() if objs}
        
        userdata.task = (self.take_obj, self.leave_obj)
        userdata.sa_poses = self.semantic_map
        userdata.flag_start = True

        Logger.loginfo(f'FINAL TASK: {userdata.task}')
        return 'continue'
