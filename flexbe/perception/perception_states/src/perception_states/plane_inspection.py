#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from plane_inspection_interfaces.srv import InspectPlane, InspectPlaneRequest

class PlaneInspectionState(EventState):
    '''
    This state is used to call the object recognition pipeline and detect objects in a scene.

    #> object_id_list          string[] ; List of detected object ids. If asked for a single object, list contains one entry.
    #> object_pose_list        PoseStamped[] ; List of detected object poses. If asked for a single object, list contains one entry.
    #> object_dim_list         Point32[] ; List of detected object dimensions. If asked for a single object list contains one entry.
    ># object_id_list          string[] ; List of detected object ids. If asked for a single object, list contains one entry.
    ># object_pose_list        PoseStamped[] ; List of detected object poses. If asked for a single object, list contains one entry.
    ># object_dim_list         Point32[] ; List of detected object dimensions. If asked for a single object list contains one entry.

    <= objects_found 		found some objects
    <= no_objects_found     no objects were found
    <= failed 				failed at some point


    '''

    def __init__(self, plane_min_x, plane_min_y, eps, min_height, max_height):
        # Typically used to declare variables and configure proxies. The state interface is defined by calling the
        # constructor of the superclass.
        super(PlaneInspectionState, self).__init__(outcomes=['done', 'failed'])


        self._pi_service_topic = "/plane_inspection/inspect_plane"
        self.min_height = min_height
        self.max_height = max_height
        self.plane_min_x = plane_min_x
        self.plane_min_y = plane_min_y
        self.eps = eps

    def execute(self, userdata):
        # This function is called periodically while the state is active. Main purpose is to check state conditions and
        # trigger a corresponding outcome. If no outcome is returned, the state will stay active.

        try:
            Logger.loginfo('Waiting for plane inspection Service')

            rospy.wait_for_service(self._pi_service_topic)
            self._client = rospy.ServiceProxy(self._pi_service_topic, InspectPlane)

            request = InspectPlaneRequest()

            request.plane_min_size_x = self.plane_min_x
            request.plane_min_size_y = self.plane_min_y
            request.min_height = self.min_height
            request.max_height = self.max_height
            request.eps = self.eps

            response = self._client(request)
            #TODO do someting usefull
            return "done"

        except Exception as e:
            Logger.logwarn("Plane Inspection failed %s" % (e))
            return "failed"


    def on_start(self):
        # This event is triggered when the behavior is started. If possible, it is generally better to initialize used
        # resources in the constructor because if anything failed, the behavior would not even be started.
        pass
