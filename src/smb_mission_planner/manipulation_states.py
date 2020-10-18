#!/usr/bin/env python2
import rospy
import smach
from smb_mission_planner.utils import MoveItPlanner
from smb_mission_planner.base_state_ros import BaseStateRos
"""
Here define all the navigation related states
"""


class JointsConfigurationVisitor(BaseStateRos):
    """
    This state keeps an internal database of joints configurations (from param server)
    and at each execution steps commands the next configuration. Once the last is reached,
    it starts looping
    Example usage: the arm has an eye-in-hand and needs to scan the scene from different viewpoints
    """
    def __init__(self, ns=""):
        BaseStateRos.__init__(self,
                              outcomes=['Completed', 'Failure'],
                              input_keys=['reset'],
                              ns=ns)

        self.joints_configurations = self.get_scoped_param("joints_configurations")
        self.n_configurations = len(self.joints_configurations)
        rospy.loginfo("Parsed {} joints configurations: {}".format(self.n_configurations, self.joints_configurations))

        self.planner = MoveItPlanner()
        self.idx = 0

    def execute(self, ud):

        if self.idx == self.n_configurations:
            rospy.logwarn("No more configurations to visit! Looping")
            self.idx = 0

        rospy.loginfo("Visiting configuration {}: {}".format(self.idx, self.joints_configurations[self.idx]))
        success = self.planner.reach_joint_angles(self.joints_configurations[self.idx], tolerance=0.01)
        self.idx += 1
        if success:
            return 'Completed'
        else:
            return 'Failure'


class MoveItNamedPositionReachingReaching(BaseStateRos):
    """
    In this state the arm reaches a prerecorded and named configuration saved in the moveit config package
    (see the <robot_name>.sdf or explore the package through moveit_setup_assistant)
    """
    def __init__(self, target, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Failure'], ns=ns)
        self.planner = MoveItPlanner()
        self.target = target

    def execute(self, _):
        rospy.loginfo("Reaching named position: {}".format(self.target))
        success = self.planner.reach_named_position(self.target)
        if success:
            return 'Completed'
        else:
            return 'Failure'


class MoveItHome(MoveItNamedPositionReachingReaching):
    """ Assumes there exist an home configuration """
    def __init__(self, ns=""):
        MoveItNamedPositionReachingReaching.__init__(self, "home", ns=ns)


class MoveItVertical(MoveItNamedPositionReachingReaching):
    """ Assumes there exist a retract configuration """
    def __init__(self, ns=""):
        MoveItNamedPositionReachingReaching.__init__(self, "vertical", ns=ns)



