#!/usr/bin/env python2
import rospy
from smb_mission_planner.base_state_ros import BaseStateRos
from smb_mission_planner.srv import DetectObjectRequest, DetectObject
"""
Here define all the detection related states
"""


class ObjectDetection(BaseStateRos):
    """
    Basic detection states which will output to retry until the maximum number of allowed failure is reached
    """
    def __init__(self, max_num_failure=1, ns=""):
        BaseStateRos.__init__(self,
                              outcomes=['Completed', 'Failure', 'Retry'],
                              input_keys=['reset'],
                              ns=ns)

        self.current_failures = 0
        self.max_num_failures = max_num_failure

    def execute(self, ud):

        success = self.detection_implementation()

        if success:
            return 'Completed'
        elif self.current_failures < self.max_num_failures:
            return 'Retry'
        else:
            return 'Failure'

    def detection_implementation(self):
        raise NotImplementedError("This method needs to be implemented by the derived class")


class ObjectDetectionWithService(ObjectDetection):
    def __init__(self, max_num_failure, ns=""):
        ObjectDetection.__init__(self, max_num_failure=max_num_failure, ns=ns)
        self.detection_service_name = self.get_scoped_param("detection_service_name")
        self.detection_service = rospy.ServiceProxy(self.detection_service_name, DetectObject)
        rospy.loginfo("Calling detection service with name: {}".format(self.detection_service_name))

    def detection_implementation(self):
        try:
            self.detection_service.wait_for_service(timeout=10.0)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return False

        req = DetectObjectRequest()
        self.detection_service.call(req)
