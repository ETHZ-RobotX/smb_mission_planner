#!/usr/bin/env python2
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandActionGoal

from smb_mission_planner.utils import MoveItPlanner
from smb_mission_planner.base_state_ros import BaseStateRos
from smb_mission_planner.utils.ros_utils import switch_ros_controller

"""
Here define all the navigation related states
"""


class RosControlPoseReaching(BaseStateRos):
    """
    Switch and send target poses to the controller manager
    """
    def __init__(self, ns=""):
        BaseStateRos.__init__(self,
                              outcomes=['Completed', 'Failure'],
                              input_keys=['reset'],
                              ns=ns)

        self.controller_name = self.get_scoped_param("controller_name")
        self.manager_namespace = self.get_scoped_param("manager_namespace")
        self.whitelist = self.get_scoped_param("whitelist")

    def do_switch(self):
        return switch_ros_controller(controller_name=self.controller_name,
                                     manager_namespace=self.manager_namespace,
                                     whitelist=self.whitelist)

    def execute(self, ud):
        rospy.logwarn("This method needs to be implemented yet. Use 'do_switch' to switch to the desired controller.")
        return 'Completed'


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


class MoveItPoseReaching(BaseStateRos):
    """
    Reach an end effector pose using moveit
    """
    def __init__(self, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Failure'], ns=ns, input_keys=['target'])
        self.planner = MoveItPlanner()

    def execute(self, ud):
        if ud.target is None:
            rospy.logerr("The target pose is None")
            return 'Failure'

        if not isinstance(ud.target, PoseStamped):
            rospy.logerr("The target pose is not of type geometry_msgs.PoseStamped")
            return 'Failure'

        success = self.planner.reach_cartesian_pose(ud.target)
        if success:
            return 'Completed'
        else:
            rospy.logerr("Failed to reach the target pose")
            return 'Failure'


class MoveItNamedPositionReaching(BaseStateRos):
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


class MoveItHome(MoveItNamedPositionReaching):
    """ Assumes there exist an home configuration """
    def __init__(self, ns=""):
        MoveItNamedPositionReaching.__init__(self, "home", ns=ns)


class MoveItVertical(MoveItNamedPositionReaching):
    """ Assumes there exist a retract configuration """
    def __init__(self, ns=""):
        MoveItNamedPositionReaching.__init__(self, "vertical", ns=ns)


class GripperControl(BaseStateRos):
    """
    This state controls the gripper through the GripperCommandAction
    """
    def __init__(self, ns=""):
        BaseStateRos.__init__(self,
                              outcomes=['Completed', 'Failure'],
                              ns=ns)

        self.position = self.get_scoped_param("position")
        self.max_effort = self.get_scoped_param("max_effort")
        self.tolerance = self.get_scoped_param("tolerance")
        self.server_timeout = self.get_scoped_param("timeout")

        self.gripper_cmd = GripperCommandActionGoal()
        self.gripper_cmd.header.stamp = rospy.get_rostime()
        self.gripper_cmd.goal.command.position = self.position
        self.gripper_cmd.goal.command.max_effort = self.max_effort

        self.gripper_action_name = self.get_scoped_param("gripper_action_name")
        self.gripper_client = actionlib.ActionClient(self.gripper_action_name, GripperCommandAction)

    def execute(self, ud):

        if not self.gripper_client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr("Timeout exceeded while waiting for {} server".format(self.gripper_action_name))
            return 'Failure'

        handle = self.gripper_client.send_goal(self.gripper_cmd)
        result = handle.get_result().result

        error = abs(result.position - self.position)
        tolerance_met = error < self.tolerance

        success = False
        if result is None:
            rospy.logerr("None received from the gripper server")
            success = False
        elif result.stalled and not tolerance_met:
            rospy.logerr("Gripper stalled and not moving, position error {} is larger than tolerance".format(error))
            success = False
        elif result.stalled and tolerance_met:
            rospy.logerr("Gripper stalled and not moving, position error {} is smaller than tolerance".format(error))
            success  = True
        elif result.reached_goal:
            rospy.loginfo("Gripper successfully reached the goal")
            success = True

        if success:
            return 'Failure'
        else:
            return 'Completed'
