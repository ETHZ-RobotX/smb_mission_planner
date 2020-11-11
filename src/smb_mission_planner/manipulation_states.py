#!/usr/bin/env python2
import rospy
import tf2_ros
import actionlib
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandActionGoal

from smb_mission_planner.utils import MoveItPlanner
from smb_mission_planner.base_state_ros import BaseStateRos
from smb_mission_planner.utils.ros_utils import switch_ros_controller
from smb_mission_planner.utils import rocoma_utils

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


class EndEffectorRocoControl(BaseStateRos):
    """
    In this state the end effector pose is controlled by a roco controller.
    This state provides utility methods to parse poses from the mission file and query
    for the current end effector pose.
    """
    def __init__(self, ns, outcomes=['Completed', 'Aborted']):
        BaseStateRos.__init__(self, outcomes=outcomes, ns=ns)
        self.timeout = self.get_scoped_param("timeout")
        self.controller = self.get_scoped_param("roco_controller")
        self.controller_manager_namespace = self.get_scoped_param("controller_manager_namespace")
        self.ee_frame = self.get_scoped_param("end_effector_frame_id")
        self.reference_frame = self.get_scoped_param("reference_frame_id")
        self.sub_controller = self.get_scoped_param("sub_roco_controller", safe=False)
        self.sub_controller_manager_namespace = self.get_scoped_param("sub_controller_manager_namespace", safe=False)

        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def switch_controller(self):
        success = rocoma_utils.switch_roco_controller(self.controller,
                                                      ns=self.controller_manager_namespace)
        if success is False:
            return success
        if self.sub_controller is not None and self.sub_controller_manager_namespace is not None:
            success = rocoma_utils.switch_roco_controller(self.sub_controller,
                                                          ns=self.sub_controller_manager_namespace)
        return success

    def get_end_effector_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.reference_frame, self.ee_frame, rospy.Time())
            ee_pose = PoseStamped()
            ee_pose.header.frame_id = self.reference_frame
            ee_pose.pose.position.x = trans.transform.translation.x
            ee_pose.pose.position.y = trans.transform.translation.y
            ee_pose.pose.position.z = trans.transform.translation.z
            ee_pose.pose.orientation.x = trans.transform.rotation.x
            ee_pose.pose.orientation.y = trans.transform.rotation.y
            ee_pose.pose.orientation.z = trans.transform.rotation.z
            ee_pose.pose.orientation.w = trans.transform.rotation.w
            return ee_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logerr("Failed to query ee transform: {}".format(exc))
            return None

    @staticmethod
    def parse_pose(pose_dictionary):
        if 't' not in pose_dictionary.keys() or 'q' not in pose_dictionary.keys():
            rospy.logerr("poses param is ill-formed")
            return None

        if len(pose_dictionary['t']) != 3 or len(pose_dictionary['q']) != 4:
            rospy.logerr("poses param is ill-formed")
            return None

        q_arr = np.array(pose_dictionary['q'])
        norm = np.linalg.norm(q_arr)
        if norm > 1.01 or norm < 0.99:
            rospy.logerr("pose quaternion is not normalized")
            return None

        pose = PoseStamped()
        pose.pose.position.x = pose_dictionary['t'][0]
        pose.pose.position.y = pose_dictionary['t'][1]
        pose.pose.position.z = pose_dictionary['t'][2]
        pose.pose.orientation.x = pose_dictionary['q'][0]
        pose.pose.orientation.y = pose_dictionary['q'][1]
        pose.pose.orientation.z = pose_dictionary['q'][2]
        pose.pose.orientation.w = pose_dictionary['q'][3]
        return pose

    def execute(self, ud):
        raise NotImplementedError("The execute method ot the EndEffectorRocoControl must be implemented yet.")
