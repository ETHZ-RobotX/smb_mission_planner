#!/usr/bin/env python2
import smach
import rospy
from os.path import join

from smb_mission_planner.utils import ros_utils


class BaseStateRos(smach.State):
    """
    Base state adding minimal ROS functionality to the basic smach state
    """
    def __init__(self, outcomes=['Completed', 'Failure', 'Retry'], input_keys=[], output_keys=[], ns=""):
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.namespace = ns

    def get_scoped_param(self, param_name):
        named_param = join(self.namespace, param_name)
        return ros_utils.get_param_safe(named_param)

    def execute(self, ud):
        raise NotImplementedError("execute() needs to be implemented by the derived class")


if __name__ == "__main__":

    class TestState(BaseStateRos):
        def __init__(self, ns=""):
            BaseStateRos.__init__(self, outcomes=['Completed', 'Failure'], ns=ns)

        def execute(self, ud):
            try:
                self.get_scoped_param("my_param")
            except NameError as exc:
                rospy.logerr(exc)
                return 'Failure'
            return 'Completed'

    rospy.init_node("base_state_ros_test")
    rospy.set_param("TEST/my_param", 1)

    state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
    with state_machine:
        smach.StateMachine.add('TEST', TestState(ns="TEST"),
                               transitions={'Completed': 'Success',
                                            'Failure': 'Failure'})
    # Execute state machine.
    outcome = state_machine.execute()
