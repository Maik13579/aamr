#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from nav_behaviors.navigate_sm import navigateSM
from robot_control_behaviors.look_to_point_sm import look_to_pointSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Sep 30 2023
@author: Maik Knof
'''
class test_1SM(Behavior):
	'''
	move to table
look to table
	'''


	def __init__(self):
		super(test_1SM, self).__init__()
		self.name = 'test_1'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(look_to_pointSM, 'look_to_point')
		self.add_behavior(navigateSM, 'navigate')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:640 y:53, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.frame_id = "map"
		_state_machine.userdata.x = -3.0
		_state_machine.userdata.y = 3.0
		_state_machine.userdata.z = 0.0
		_state_machine.userdata.xq = 0.0
		_state_machine.userdata.yq = 0.0
		_state_machine.userdata.zq = 0.924
		_state_machine.userdata.wq = 0.383
		_state_machine.userdata.x_coordinate = -4.0
		_state_machine.userdata.y_coordinate = 4.0
		_state_machine.userdata.z_coordinate = 0.4
		_state_machine.userdata.duration = 2.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:162 y:193
			OperatableStateMachine.add('navigate',
										self.use_behavior(navigateSM, 'navigate'),
										transitions={'finished': 'look_to_point', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'frame_id': 'frame_id', 'x': 'x', 'y': 'y', 'z': 'z', 'xq': 'xq', 'yq': 'yq', 'zq': 'zq', 'wq': 'wq'})

			# x:383 y:75
			OperatableStateMachine.add('look_to_point',
										self.use_behavior(look_to_pointSM, 'look_to_point'),
										transitions={'finished': 'finished', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'x_coordinate': 'x_coordinate', 'y_coordinate': 'y_coordinate', 'z_coordinate': 'z_coordinate', 'frame_id': 'frame_id', 'duration': 'duration'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
