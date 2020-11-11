import rospy
import copy
from std_srvs.srv import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import TransformStamped

from cpt_pointlaser_msgs.srv import HighAccuracyLocalization, HighAccuracyLocalizationRequest
from mabi_speedy12_msgs.srv import DesiredWrenchCurrentEEFrameReference, DesiredWrenchCurrentEEFrameReferenceRequest
from smb_mission_planner.base_state_ros import BaseStateRos
from smb_mission_planner.navigation_states import SingleNavGoalState
from smb_mission_planner.manipulation_states import EndEffectorRocoControl
from smb_mission_planner.utils.navigation_utils import nav_goal_from_path


class MissionStarter(BaseStateRos):
    """
    Starts the mission by recording the task path (from building model) in the global
    context (accessible by all other states)
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_subscriber = rospy.Subscriber(self.path_topic_name, Path, self.path_callback,
                                                queue_size=1)
        self.timeout = self.get_scoped_param("timeout")
        self.data_received = False
        self.callback_active = True

    def path_callback(self, msg):
        if self.callback_active:
            self.set_context_data("grinding_path", msg)
            self.data_received = True
            self.callback_active = False

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        self.callback_active = True
        elapsed = 0.0
        while not rospy.is_shutdown() and elapsed < self.timeout:
            if not self.data_received:
                rospy.sleep(1.0)
                elapsed += 1.0
            else:
                return 'Completed'

        rospy.logerr("Timeout elapsed while waiting for the input path")
        return 'Aborted'


class NavigateToWall(SingleNavGoalState):
    """
    Uses the task path to compute an intermediate target for the base only.
    This is defined to be in front of the wall to a predefined distance in the direction
    of the normal to the plane defined by the path. Additional params for base navigation
    are defined in the base class SingleNavGoalState
    """
    def __init__(self, ns):
        SingleNavGoalState.__init__(self, ns=ns)
        self.distance_from_wall = self.get_scoped_param("distance_from_wall")
        self.normal_direction = self.get_scoped_param("normal_direction")

    def execute(self, userdata):
        if self.default_outcome:
            return self.default_outcome

        path = self.get_context_data("grinding_path")
        nav_goal = nav_goal_from_path(path,
                                      distance_from_wall=self.distance_from_wall,
                                      normal_direction=self.normal_direction,
                                      frame_id=path.header.frame_id)

        success = self.reach_goal(nav_goal)
        if success:
            rospy.loginfo("Reached offset navigation goal next to wall")
            return 'Completed'
        else:
            rospy.loginfo("Failed to reach navigation goal next to wall")
            return 'Aborted'


class HALInitialArmPositioning(EndEffectorRocoControl):
    """
    Positions the end-effector to an initial pose required for the HAL routine. The pose is
    read from the parameter server and is expressed in the marker frame.
    """
    def __init__(self, ns):
        HALInitialArmPositioning.__init__(self, ns=ns)
        self.target_offset = self.parse_pose(self.get_scoped_param("target_offset"))

        if not self.target_offset:
            rospy.logerr("Failed to parse the target end-effector pose for initial positioning in HAL routine")
            return 'Aborted'

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        if not self.switch_controller():
            rospy.logerr(
                "InitialPositioning failed: failed to switch controller")
            return 'Aborted'

        goal_path = Path()
        goal_path.header.stamp = rospy.get_rostime()
        goal_path.header.frame_id = self.reference_frame

        # Get current end effector pose.
        current_pose = self.get_end_effector_pose()
        if not current_pose:
            return 'Aborted'

        # Transform the target pose from the marker frame to the reference frame.
        tf_ref_marker = TransformStamped()
        try:
            tf_ref_marker = self.tf_buffer.lookup_transform(self.reference_frame,  # target frame
                                                            "marker",              # source frame
                                                            rospy.Time(0),         # tf at first available time
                                                            rospy.Duration(3))     # wait for 3 seconds
        except Exception as exc:
            rospy.logerr(exc)
            return 'Aborted'
        goal_pose = tf2_geometry_msgs.do_transform_pose(self.target_offset, tf_ref_marker)
        goal_pose.header.frame_id = self.reference_frame

        # same time, let mpc decide the timing
        current_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(current_pose)

        goal_pose.header.stamp = rospy.get_rostime() + rospy.Duration(self.timeout) - rospy.Duration(1.0)
        goal_path.poses.append(goal_pose)

        rospy.loginfo("Publishing path for initial positioning of the end-effector for the HAL routine.")
        self.path_publisher.publish(goal_path)

        rospy.loginfo("Waiting {} before switch".format(self.timeout))
        rospy.sleep(self.timeout)
        return 'Completed'


class ArmPosesVisitor(EndEffectorRocoControl):
    """
    Visits a sequence of poses for the end effector. Whenever this state is accessed again (because of a loop
    in the state machine), the next pose is sent as a target. Since the receiving controller does not implement a
    service or action, it waits for a fixed time, then it assumes that the pose has been reached.
    The poses are read in the format of a list with the keys t and q for translation and quaternion
    (order qx, qy, qz, qw) respectively. E.g:
     - {t : [0.0, 0.0, 0.0],
        q:  [0.0, 0.0, 0.0, 1.0]}
     - {t : [0.1, 1.0, 2.0],
        q:  [0.0, 0.0, 0.0, 1.0]}
     - ...
    NOTE: These poses are *relative* to the previous pose of the end effector.

    """
    def __init__(self, ns):
        EndEffectorRocoControl.__init__(self, ns=ns, outcomes=['PoseReached', 'NoMorePoses', 'Aborted'])
        self.poses = self.get_scoped_param("poses")
        self.poses_ros = []
        self.pose_idx = 0
        self.num_poses = len(self.poses)

    def check_poses(self):
        for pose in self.poses:
            self.poses_ros.append(self.parse_pose(pose))
            if not self.poses[-1]:
                return False

        return True

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        if not self.check_poses():
            rospy.logerr("The poses list is ill-formed, aborting...")
            return 'Aborted'

        success = self.switch_controller()

        if not success:
            rospy.logerr("ArmPoseVisitor failed")
            return 'Aborted'

        if self.pose_idx >= self.num_poses:
            rospy.loginfo("No more target poses!")
            return 'NoMorePoses'

        goal_path = Path()
        goal_path.header.stamp = rospy.get_rostime()
        goal_path.header.frame_id = self.reference_frame

        # get current end effector pose
        current_pose = self.get_end_effector_pose()
        if not current_pose:
            return 'Aborted'

        # Transform the relative pose read from the config file to a pose in the reference frame.
        goal_pose = tf2_geometry_msgs.do_transform_pose(self.poses_ros[self.pose_idx], current_pose)
        goal_pose.header.frame_id = self.reference_frame

        # same time, let mpc decide the timing
        current_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(current_pose)

        goal_pose.header.stamp = rospy.get_rostime() + rospy.Duration(self.timeout) - rospy.Duration(1.0)
        goal_path.poses.append(goal_pose)

        rospy.loginfo("Publishing path with pose #{}".format(self.pose_idx))
        self.path_publisher.publish(goal_path)

        rospy.loginfo("Waiting {} before switch".format(self.timeout))
        rospy.sleep(self.timeout)
        self.pose_idx += 1
        return 'PoseReached'


class HALDataCollection(BaseStateRos):
    """
    Triggers the data collection on the HAL routine side.
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        service_name = self.get_scoped_param("service_name")
        self.timeout = self.get_scoped_param("timeout")
        self.data_collection_service_client = rospy.ServiceProxy(service_name, Empty)

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        try:
            self.data_collection_service_client.wait_for_service(self.timeout)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return 'Aborted'

        self.data_collection_service_client.call()
        return 'Completed'


class HALOptimization(BaseStateRos):
    """
    Triggers the HAL Optimization routine and send the received pose update to confusor for the state
    estimation update
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        hal_optimization_service_name = self.get_scoped_param("hal_optimization_service_name")
        self.hal_timeout = self.get_scoped_param("hal_timeout")
        self.hal_service_client = rospy.ServiceProxy(hal_optimization_service_name, HighAccuracyLocalization)

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        try:
            self.hal_service_client.wait_for_service(self.hal_timeout)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return 'Aborted'

        hal_req = HighAccuracyLocalizationRequest()
        hal_res = self.hal_service_client.call(hal_req)
        rospy.loginfo("Update base pose in world frame is: {}".format(hal_res.corrected_base_pose_in_world))

        return 'Completed'


class InitializeGrinding(BaseStateRos):
    """
    Starts the grinder
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.timeout = self.get_scoped_param("timeout")

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        rospy.logwarn("The execution of the InitializeGrinding state needs to be implemented. Sleeping for 3.0 s")
        rospy.sleep(3.0)
        return 'Completed'


class InitialPositioning(EndEffectorRocoControl):
    """
    Parse an offset from param server and uses this and the first pose in the path to
    define the target pose for the end effector
    """
    def __init__(self, ns):
        EndEffectorRocoControl.__init__(self, ns=ns)
        offset = self.get_scoped_param("offset")

        # Transformation from the first pose in the grinding path and a initializing pose
        # for the end effector
        self.offset_pose = self.parse_pose(offset)

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        if not self.switch_controller():
            rospy.logerr("InitialPositioning failed: failed to switch controller")
            return 'Aborted'

        goal_path = Path()
        goal_path.header.stamp = rospy.get_rostime()
        goal_path.header.frame_id = self.reference_frame

        # get current end effector pose
        current_pose = self.get_end_effector_pose()
        if not current_pose:
            return 'Aborted'

        # the goal pose is expressed in the reference frame with an offset wrt the first pose
        # in the grinding path

        # extract tf if needed from grinding path frame to the reference frame
        grinding_path = copy.deepcopy(self.get_context_data("grinding_path"))
        if not grinding_path:
            rospy.logerr("Failed to get grinding path from global context")
            return 'Aborted'

        # naming conventions:
        # w : the reference frame for the stored grinding path
        # ref : the reference frame for the controller path (from param file)
        # start : the first pose in the grinding path
        # ee : the reference frame
        # T_start_ee : the offset of the ee in the path frame

        # The goal is T_ref_ee = T_ref_w * T_w_start * T_start_ee

        tf_w_start = TransformStamped()
        tf_ref_w = TransformStamped()
        if grinding_path.header.frame_id is not self.reference_frame:
            try:
                tf_ref_w = self.tf_buffer.lookup_transform(self.reference_frame,            # target frame
                                                           grinding_path.header.frame_id,   # source frame
                                                           rospy.Time(0),                   # tf at first available time
                                                           rospy.Duration(3))               # wait for 3 seconds
            except Exception as exc:
                rospy.logerr(exc)
                return 'Aborted'
        else:
            tf_ref_w.transform.translation.x = 0
            tf_ref_w.transform.translation.y = 0
            tf_ref_w.transform.translation.z = 0
            tf_ref_w.transform.rotation.x = 0
            tf_ref_w.transform.rotation.y = 0
            tf_ref_w.transform.rotation.z = 0
            tf_ref_w.transform.rotation.w = 1.0

        tf_w_start.transform.translation.x = grinding_path.poses[0].pose.position.x
        tf_w_start.transform.translation.y = grinding_path.poses[0].pose.position.y
        tf_w_start.transform.translation.z = grinding_path.poses[0].pose.position.z
        tf_w_start.transform.rotation.x = grinding_path.poses[0].pose.orientation.x
        tf_w_start.transform.rotation.y = grinding_path.poses[0].pose.orientation.y
        tf_w_start.transform.rotation.z = grinding_path.poses[0].pose.orientation.z
        tf_w_start.transform.rotation.w = grinding_path.poses[0].pose.orientation.w

        # First transform the offset expressed in the first pose frame in the reference frame of the first pose
        # path message (here denoted w, this is the transform equal to the path message).
        # Then convert the resulting pose in the reference frame of the ee goal (here denoted ref)
        tf_ref_ee = tf2_geometry_msgs.do_transform_pose(tf2_geometry_msgs.do_transform_pose(self.offset_pose,
                                                                                            tf_w_start),
                                                        tf_ref_w)
        goal_pose = tf_ref_ee
        goal_pose.header.frame_id = self.reference_frame
        goal_pose.header.stamp = rospy.get_rostime()

        # same time, let mpc decide the timing
        current_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(current_pose)

        goal_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(goal_pose)

        rospy.loginfo("Publishing path for initial positioning.")
        self.path_publisher.publish(goal_path)

        rospy.loginfo("Waiting {} before switch".format(self.timeout))
        rospy.sleep(self.timeout)
        return 'Completed'


class MoveIntoContact(BaseStateRos):
    """
    Move the end effector into the wall commanding a desired interaction wrench
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.set_wrench_service_name = self.get_scoped_param("wrench_service_name")
        self.reset_reference = self.get_scoped_param("reset_reference")
        self.fx = self.get_scoped_param("force/x")
        self.fy = self.get_scoped_param("force/y")
        self.fz = self.get_scoped_param("force/z")
        self.set_wrench_service_client = rospy.ServiceProxy(self.set_wrench_service_name,
                                                            DesiredWrenchCurrentEEFrameReference)

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        try:
            self.set_wrench_service_client.wait_for_service(3.0)
        except rospy.ROSException as exc:
            rospy.logerr("Failed to call set wrench service: {}".format(exc))
            return 'Aborted'

        req = DesiredWrenchCurrentEEFrameReferenceRequest()
        req.wrench.force.x = self.fx
        req.wrench.force.y = self.fy
        req.wrench.force.z = self.fz
        req.wrench.torque.x = 0.0
        req.wrench.torque.y = 0.0
        req.wrench.torque.z = 0.0
        req.resetReference = self.reset_reference
        res = self.set_wrench_service_client.call(req)
        rospy.loginfo("{} responded with message: {}".format(self.set_wrench_service_name, res.message))
        if res.success:
            return 'Completed'
        else:
            return 'Aborted'


class FollowGrindingPath(EndEffectorRocoControl):
    """
    Does the grinding
    """
    def __init__(self, ns):
        EndEffectorRocoControl.__init__(self, ns=ns)

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        grinding_path = self.get_context_data("grinding_path")
        if not grinding_path:
            rospy.logerr("Failed to retrieve grinding_path from global context")
            return 'Aborted'

        current_time = rospy.get_rostime()
        grinding_path_transformed = copy.deepcopy(grinding_path)
        grinding_path_transformed.header.frame_id = self.reference_frame

        if grinding_path.header.frame_id is not self.reference_frame:
            rospy.loginfo("grinding path is in {} frame. Converting it to {} reference frame".
                          format(grinding_path.header.frame_id,
                                 self.reference_frame))
            try:
                transform = self.tf_buffer.lookup_transform(self.reference_frame,            # target frame
                                                            grinding_path.header.frame_id,   # source frame
                                                            rospy.Time(0),                   # tf at first available time
                                                            rospy.Duration(3))               # wait for 3 seconds
                for idx, pose in enumerate(grinding_path.poses):
                    grinding_path_transformed.poses[idx] = tf2_geometry_msgs.do_transform_pose(pose, transform)
                    grinding_path_transformed.poses[idx].header.stamp = current_time

            except Exception as exc:
                rospy.logerr("Failed lookup: {}".format(exc))
                return 'Aborted'

        grinding_path_transformed.poses.insert(0, self.get_end_effector_pose())
        grinding_path_transformed.poses[0].header.stamp = current_time
        self.path_publisher.publish(grinding_path_transformed)
        rospy.loginfo("Waiting {} before switch".format(self.timeout))
        rospy.sleep(self.timeout)
        return 'Completed'


class AsBuiltSensing(EndEffectorRocoControl):
    """
    Performs... TODO(hermann)
    """
    def __init__(self, ns):
        EndEffectorRocoControl.__init__(self, ns=ns)
        self.scanning_pose = self.get_scoped_param("scanning_pose")

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        if not self.switch_controller():
            rospy.logerr("AsBuiltSensing failed: failed to switch controller")
            return 'Aborted'

        goal_path = Path()
        goal_path.header.stamp = rospy.get_rostime()
        goal_path.header.frame_id = self.reference_frame

        # get current end effector pose
        current_pose = self.get_end_effector_pose()
        if not current_pose:
            return 'Aborted'

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.reference_frame
        goal_pose.header.stamp = rospy.get_rostime()
        goal_pose.pose = self.parse_pose(self.scanning_pose)
        if not goal_pose.pose:
            rospy.logerr("Failed to parse the scanning pose")
            return 'Aborted'

        # same time, let mpc decide the timing
        current_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(current_pose)

        goal_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(goal_pose)

        rospy.loginfo("Publishing path for as-built sensing.")
        self.path_publisher.publish(goal_path)

        rospy.loginfo("Waiting {} before switch".format(self.timeout))
        rospy.sleep(self.timeout)
        return 'Completed'
