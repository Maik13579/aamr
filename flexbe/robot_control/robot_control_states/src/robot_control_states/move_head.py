#!/usr/bin/env python
import time

from actionlib_msgs.msg import GoalStatus

from control_msgs.msg import PointHeadAction

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

HEAD_ACTION_TOPIC = "/head_controller/point_head_action"

class SendHeadActionState(EventState):
    """
    This class sends the action to pan/tilt the head to robot.

    ># head_action_goal	    PointHeadActionGoal		Msg to be sent to pan/tilt head.

    <= success 				Head is in pan/tilt position.
    <= failed				Could not complete the action.
    """

    def __init__(self, timeout=5):
        # Constructor to initialize the state.
        super(SendHeadActionState, self).__init__(
            outcomes=["success", "failed", "timeout"], input_keys=["head_action_goal"]
        )
        self._topic = None
        self._client = None

        # It may happen that the action client fails to send the action goal.
        self._error = False

        self.start_time = None
        self.timeout = timeout

    def on_start(self):
        # This method is called when the behavior is started.

        # Create the action client when building the behavior.
        # This will cause the behavior to wait for the client before starting execution
        # and will trigger a timeout error if it is not available.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        self._topic = HEAD_ACTION_TOPIC
        self._client = ProxyActionClient(
            {self._topic: PointHeadAction}
        )  # pass required clients as dict (topic: type)

    def on_enter(self, userdata):

        self.start_time = time.time()
        # Send the goal.
        self._error = False  # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, userdata.head_action_goal.goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, only print warnings are printed, not errors.
            Logger.logwarn("Failed to send the change head state command:\n%s" % str(e))
            self._error = True

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return "failed"

        # Check if the action has been finished.
        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)
            if status == GoalStatus.SUCCEEDED:
                return "success"
            elif status in [
                GoalStatus.PREEMPTED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ]:
                Logger.logwarn("Failed to pan/tilt head: %s" % str(status))
                return "failed"

        if (time.time() - self.start_time) > self.timeout:
            return "timeout"

    # If the action has not yet finished, no outcome will be returned and the state stays active.

    def cancel_head_action(self):
        # Check if the client is still active.
        if self._client.is_active(self._topic):
            if not self._client.has_result(self._topic):
                self._client.cancel(self._topic)
                Logger.loginfo("Cancelled active head pan/tilt action goal.")

    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up the resources.

        self.cancel_head_action()

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.

        self.cancel_head_action()
