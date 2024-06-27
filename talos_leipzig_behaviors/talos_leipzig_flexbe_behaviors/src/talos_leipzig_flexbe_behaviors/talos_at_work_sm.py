#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from talos_leipzig_flexbe_states.move_base_state import MoveBaseState
from talos_leipzig_flexbe_states.refbox_parser import RefBoxParserState
from talos_leipzig_flexbe_states.refbox_planner import RefBoxPlannerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 27 2024
@author: Altzi Tsanko
'''
class talos_at_workSM(Behavior):
	'''
	Simple Navigation RoboCup@Work
	'''


	def __init__(self):
		super(talos_at_workSM, self).__init__()
		self.name = 'talos_at_work'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:706 y:23, x:183 y:310
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:115 y:63
			OperatableStateMachine.add('Refbox_Parser',
										RefBoxParserState(),
										transitions={'continue': 'Refbox_Planner', 'error_parsing': 'failed'},
										autonomy={'continue': Autonomy.Off, 'error_parsing': Autonomy.Off},
										remapping={'task': 'task', 'sa_poses': 'sa_poses', 'flag_start': 'flag_start'})

			# x:308 y:65
			OperatableStateMachine.add('Refbox_Planner',
										RefBoxPlannerState(),
										transitions={'continue': 'move_base', 'failed': 'failed', 'test_exhausted': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'test_exhausted': Autonomy.Off},
										remapping={'task': 'task', 'sa_poses': 'sa_poses', 'flag_start': 'flag_start', 'waypoint': 'waypoint'})

			# x:528 y:96
			OperatableStateMachine.add('move_base',
										MoveBaseState(),
										transitions={'arrived': 'Refbox_Planner', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint', 'flag_start': 'flag_start'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
