#!/usr/bin/env python

import rospy
import os
import yaml
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from atwork_commander_msgs.msg import Task, Object, Workstation

'''
Created: 27/06/2024

@author: Altzi Tsanko
email: ace.tsan21@gmail.com
'''

class RefBoxParserStateV1(EventState):
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
        super(RefBoxParserStateV1, self).__init__(outcomes=['continue', 'error_parsing'], output_keys=['task', 'sa_poses', 'flag_start'])
        self.task_topic = 'atwork_commander/task'
        self.semantic_map_path = '/home/dir/sim_ws/src/talos_sim/talos_leipzig_behaviors/talos_leipzig_flexbe_states/src/talos_leipzig_flexbe_states/semantic_map/leipzig_map.yaml'

        # Subscribe to task topic
        self.task_sub = ProxySubscriberCached({self.task_topic: Task})

    def on_start(self):
        """ Load semantic map when the state machine starts. """
        try:
            with open(self.semantic_map_path, 'r') as stream:
                semantic_map = yaml.safe_load(stream)
                if isinstance(semantic_map, list):
                    self.semantic_map = {k: v for d in semantic_map for k, v in d.items()}
                else:
                    self.semantic_map = {}
                    Logger.logwarn("Unexpected semantic map format: {type(semantic_map)}")
        except yaml.YAMLError as exc:
            Logger.logerror("Error loading semantic map: {exc}")
            self.semantic_map = {}
        except Exception as exc:
            Logger.logerror("Unexpected error loading semantic map: {exc}")
            self.semantic_map = {}

    def on_enter(self, userdata):
        """ Wait for a task message to be published. """
        while not self.task_sub.has_msg(self.task_topic):
            Logger.logwarn('Refbox atwork_commander/task topic is not being published.')
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
            object_msg.Axis2: 'Axis2',
            object_msg.Bearing2: 'Bearing2',
            object_msg.S40_40_G: 'S40_40_G',
            object_msg.S40_40_B: 'S40_40_B',
            object_msg.M20: 'M20',
            object_msg.M30: 'M30',
            object_msg.M20_100: 'M20_100',
            object_msg.Housing: 'Housing',
            object_msg.Motor2: 'Motor2',
            object_msg.Spacer: 'Spacer',
            object_msg.Screwdriver: 'Screwdriver',
            object_msg.Wrench: 'Wrench',
            object_msg.Drill: 'Drill',
            object_msg.AllenKey: 'AllenKey',
        }

    def execute(self, userdata):
        """ Execute the state logic. """
        if not self.task_sub.has_msg(self.task_topic):
            return 'error_parsing'

        # Retrieve and process the last data from Object_Task topic
        task_msg = self.task_sub.get_last_msg(self.task_topic)
        self.task_sub.remove_last_msg(self.task_topic)
        self.start_state = task_msg.arena_start_state
        self.target_state = task_msg.arena_target_state

        #Logger.loginfo('Objects: {len(self.start_state)}')

        # Populate dictionaries from the topic
        for workstation in self.start_state:
            for obj in workstation.objects:
                self.take_obj[workstation.workstation_name].append(obj.object)
                rospy.loginfo("Start State: Object: %d, Target: %d, Decoy: %s", obj.object, obj.target, obj.decoy)
        for workstation in self.target_state:
            for obj in workstation.objects:
                self.leave_obj[workstation.workstation_name].append(obj.object)
                rospy.loginfo("Target State: Object: %d, Target: %d, Decoy: %s", obj.object, obj.target, obj.decoy)


        # Remove empty workstation goals
        self.take_obj = {ws: objs for ws, objs in self.take_obj.items() if objs}
        self.leave_obj = {ws: objs for ws, objs in self.leave_obj.items() if objs}
        
        userdata.task = (self.take_obj, self.leave_obj)
        userdata.sa_poses = self.semantic_map
        userdata.flag_start = True

        Logger.loginfo('FINAL TASK: {userdata.task}')
        return 'continue'
