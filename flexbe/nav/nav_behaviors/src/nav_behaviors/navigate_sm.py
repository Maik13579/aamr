#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from nav_states.create_move_base_goal import NavCreateMoveBaseGoalState
from nav_states.move_base import NavMoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Sep 30 2023
@author: Maik Knof
'''
class navigateSM(Behavior):
	'''
	Navigate to pose using move_base
	'''


	def __init__(self):
		super(navigateSM, self).__init__()
		self.name = 'navigate'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:704 y:76, x:362 y:160
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['frame_id', 'x', 'y', 'z', 'xq', 'yq', 'zq', 'wq'])
		_state_machine.userdata.frame_id = ""
		_state_machine.userdata.x = 0.0
		_state_machine.userdata.y = 0.0
		_state_machine.userdata.z = 0.0
		_state_machine.userdata.xq = 0.0
		_state_machine.userdata.yq = 0.0
		_state_machine.userdata.zq = 0.0
		_state_machine.userdata.wq = 0.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:99 y:70
			OperatableStateMachine.add('create_goal',
										NavCreateMoveBaseGoalState(),
										transitions={'success': 'navigate', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'frame_id': 'frame_id', 'x': 'x', 'y': 'y', 'z': 'z', 'xq': 'xq', 'yq': 'yq', 'zq': 'zq', 'wq': 'wq', 'goal': 'goal'})

			# x:439 y:70
			OperatableStateMachine.add('navigate',
										NavMoveBaseState(),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
