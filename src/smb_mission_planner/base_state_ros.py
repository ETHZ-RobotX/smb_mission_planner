#!/usr/bin/env python2
import smach
import rospy
from os.path import join

from smb_mission_planner.utils import ros_utils


class StateMachineContext(object):
    def __init__(self):
        self.data = {}


global_context = StateMachineContext()


class BaseStateRos(smach.State):
    """
    Base state adding minimal ROS functionality to the basic smach state
    """
    def __init__(self, outcomes=['Completed', 'Failure', 'Retry'],
                 input_keys=[],
                 output_keys=[],
                 ns=""):
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.namespace = ns
        global global_context
        self.global_context = global_context

    def get_scoped_param(self, param_name):
        """
        Get the absolute path relative to the state namespace of the path relative to the
        state and ros namespace
        Example: TODO
        """
        print(rospy.get_namespace())
        if param_name.startswith('/'):
            named_param = join(self.namespace, param_name)
        else:
            named_param = join(rospy.get_namespace(), self.namespace, param_name)
        return ros_utils.get_param_safe(named_param)

    def execute(self, ud):
        raise NotImplementedError("execute() needs to be implemented by the derived class")

    def set_context_data(self, key, data, overwrite=False):
        """
        Set a new data field in the global context accessible to all states which
        are derived from this state
        :param key: data key
        :param data: data content
        :param overwrite: if True, let the user overwrite data if already in global context
        :return: True if read was successful
        """
        if not overwrite and key in self.global_context.data.keys():
            rospy.logwarn("Could not set data in global context. Key {} already exists".format(key))
            return False
        self.global_context.data[key] = data

    def get_context_data(self, key):
        if key in self.global_context.data.keys():
            return self.global_context.data[key]
        else:
            rospy.logwarn("Failed to retrieve [{}] from data".format(key))
            return None


if __name__ == "__main__":

    class StateA(BaseStateRos):
        def __init__(self, ns=""):
            BaseStateRos.__init__(self, outcomes=['Completed', 'Failure'], ns=ns)

        def execute(self, ud):
            try:
                self.get_scoped_param("my_param")
                value = self.get_context_data("state_b_data")
                rospy.loginfo("value is {}".format(value))
            except NameError as exc:
                rospy.logerr(exc)
                return 'Failure'
            return 'Completed'


    class StateB(BaseStateRos):
        def __init__(self, ns=""):
            BaseStateRos.__init__(self, outcomes=['Completed', 'Failure'], ns=ns)

        def execute(self, ud):
            self.set_context_data("state_b_data", 3)
            return 'Completed'


    rospy.init_node("base_state_ros_test")
    rospy.set_param("state_a/my_param", 1)

    state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
    with state_machine:
        smach.StateMachine.add('STATE_A1', StateA(ns="state_a"),
                               transitions={'Completed': 'STATE_B',
                                            'Failure': 'Failure'})

        smach.StateMachine.add('STATE_B', StateB(ns="state_b"),
                               transitions={'Completed': 'STATE_A2',
                                            'Failure': 'Failure'})

        smach.StateMachine.add('STATE_A2', StateA(ns="state_a"),
                               transitions={'Completed': 'Success',
                                            'Failure': 'Failure'})

    # Execute state machine.
    outcome = state_machine.execute()
