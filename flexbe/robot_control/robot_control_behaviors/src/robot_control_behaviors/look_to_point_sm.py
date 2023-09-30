#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from robot_control_states.create_point_head_goal import CreateHeadGoalState
from robot_control_states.move_head import SendHeadActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Sep 30 2023
@author: Maik Knof
'''
class look_to_pointSM(Behavior):
	'''
	Move the head to look to a point
	'''


	def __init__(self):
		super(look_to_pointSM, self).__init__()
		self.name = 'look_to_point'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:710 y:136, x:323 y:225, x:519 y:229
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'timeout'], input_keys=['x_coordinate', 'y_coordinate', 'z_coordinate', 'frame_id', 'duration'])
		_state_machine.userdata.x_coordinate = -4.0
		_state_machine.userdata.y_coordinate = 4.0
		_state_machine.userdata.z_coordinate = 0.5
		_state_machine.userdata.frame_id = "map"
		_state_machine.userdata.duration = 3.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:208 y:83
			OperatableStateMachine.add('create_goal',
										CreateHeadGoalState(),
										transitions={'success': 'move_head', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'x_coordinate': 'x_coordinate', 'y_coordinate': 'y_coordinate', 'z_coordinate': 'z_coordinate', 'duration': 'duration', 'frame_id': 'frame_id', 'head_action_goal': 'head_action_goal'})

			# x:468 y:82
			OperatableStateMachine.add('move_head',
										SendHeadActionState(timeout=5),
										transitions={'success': 'finished', 'failed': 'failed', 'timeout': 'timeout'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'head_action_goal': 'head_action_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
