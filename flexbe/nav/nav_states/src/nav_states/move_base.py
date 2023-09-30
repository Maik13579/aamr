# The code for this state is mostly taken from here:
# https://github.com/FlexBE/generic_flexbe_states/blob/master/flexbe_navigation_states/src/flexbe_navigation_states/move_base_state.py

from actionlib_msgs.msg import GoalStatus
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from move_base_msgs.msg import MoveBaseAction

MOVE_BASE_TOPIC = "/move_base"

class NavMoveBaseState(EventState):
    '''
    Sends a goal received in the userdata to MoveBaseAction

    ># goal         MoveBaseGoal        move_base goal

    <= success                          robot reached goal
    <= failed                           robot failed to reach goal
    '''

    def __init__(self):
        super(NavMoveBaseState, self).__init__(outcomes=["success", "failed"], input_keys=["goal"])
        self._action_topic = MOVE_BASE_TOPIC
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._success = False
        self._failed = False

    def execute(self, userdata):
        if self._success:
            return "success"
        if self._failed:
            return "failed"

        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                Logger.loginfo("Goal reached")
                self._success = True
                return "success"
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn("Navigation failed: {}".format(str(status)))
                self._failed = True
                return "failed"

    def on_enter(self, userdata):
        self._success = False
        self._failed = False

        goal = userdata.goal

        try:
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
            Logger.logwarn("Unable to send navigation goal: {}".format(str(e)))
            self._failed = True

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo("Cancelled move_base active action goal")

    def on_exit(self, userdata):
        self.cancel_active_goals()

    def on_stop(self):
        self.cancel_active_goals()