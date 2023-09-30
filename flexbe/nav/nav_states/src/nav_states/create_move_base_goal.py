# This state takes a position and orientation in x, y, z, xq, yq, zq, wq
# and constructs a MoveBaseGoal msg that can be passed to
# the move_base action.

from flexbe_core import EventState, Logger
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseGoal


class NavCreateMoveBaseGoalState(EventState):
    '''
    Constructs a MoveBaseGoal from userdata with fields frame_id, x, y, z, xq, yq, zq, wq

    ># frame_id     String          Frame id of target pose

    ># x            Float           x-coordinate of target pose
    ># y            Float           y-coordinate of target pose
    ># z            Float           z-coordinate of target pose

    ># xq           Float           x-quaternion of target pose
    ># yq           Float           y-quaternion of target pose
    ># zq           Float           z-quaternion of target pose
    ># wq           Float           w-quaternion of target pose

    #> goal         MoveBaseGoal    Target goal for move_base

    <= success                      Generation of move_base goal successful
    <= failed                       Generation of move_base goal failed
    '''

    def __init__(self):
        super(NavCreateMoveBaseGoalState, self).__init__(outcomes=["success", "failed"],
                                                         input_keys=["frame_id", "x", "y", "z",
                                                                     "xq", "yq", "zq", "wq"],
                                                         output_keys=["goal"])

        self._success = False
        self._failed = False

    def execute(self, userdata):
        if self._success:
            return "success"

        if self._failed:
            return "failed"

    def on_enter(self, userdata):
        self._success = False
        self._failed = False

        # populate MoveBaseGoal() msg with the input userdata fields
        try:
            pt = Point(x=userdata.x, y=userdata.y, z=userdata.z)
            qt = Quaternion(x=userdata.xq, y=userdata.yq, z=userdata.zq, w=userdata.wq)
            pose = Pose(position=pt, orientation=qt)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = userdata.frame_id
            goal.target_pose.pose = pose

            userdata.goal = goal

            self._success = True
            Logger.loginfo("Move Base goal created")

        except Exception as e:
            self._failed = True
            Logger.logwarn("{}".format(str(e)))
