#!/usr/bin/env python
from control_msgs.msg import PointHeadActionGoal

from flexbe_core import EventState, Logger

POINTING_FRAME = "xtion_optical_frame"

class CreateHeadGoalState(EventState):
    '''
    This class creates a PointHeadActionGoal from the values fetched from userdata.

    ># x_coordinate         double      Value x of coordinate to look at
    ># y_coordinate         double      Value y of coordinate to look at
    ># z_coordinate         double      Value z of coordinate to look at
    ># duration             double      time to reach goal (head movement speed)
    ># frame_id             string      frame_id_reference

    #> head_action_goal	    PointHeadActionGoal		Msg to be sent to pan/tilt head.

    <= success 							Head action goal message is created.
    <= failed							Could not complete the action.
    '''

    def __init__(self):
        # Constructor to initialize the state.
        super(CreateHeadGoalState, self).__init__(outcomes=['success', 'failed'],
                                                  input_keys=['x_coordinate', 'y_coordinate', 'z_coordinate', 'duration', 'frame_id'],
                                                  output_keys=['head_action_goal'])
        self._error = False

    def on_start(self):
        # This method is called when the behavior is started.
        pass  # Nothing to be done here.

    def on_enter(self, userdata):
        # When entering this state, we create the action goal for the robot to move its fingers.

        self._error = False  # make sure to reset the error state since a previous state execution might have failed
        try:
            # Create the goal.
            action_goal = PointHeadActionGoal()
            action_goal.goal.target.point.x = (userdata.x_coordinate)
            action_goal.goal.target.point.y = (userdata.y_coordinate)
            action_goal.goal.target.point.z = (userdata.z_coordinate)
            action_goal.goal.min_duration.secs = int(userdata.duration)
            action_goal.goal.min_duration.nsecs = int((userdata.duration - int(userdata.duration)) * 1000000000)
            action_goal.goal.target.header.frame_id = userdata.frame_id
            action_goal.goal.pointing_frame = POINTING_FRAME
            action_goal.goal.pointing_axis.z = 1

            userdata.head_action_goal = action_goal

        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, only warnings are printed, not errors.
            Logger.logwarn('Failed to create the head state command:\n%s' % str(e))
            self._error = True

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the state failed to create the action goal.
        if self._error:
            return 'failed'
        else:
            return 'success'

    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up the resources.

        pass  # Nothing to be done here.

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.

        pass  # Nothing to be done here.
