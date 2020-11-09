import rospy
import copy
from std_srvs.srv import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from cpt_pointlaser_msgs.srv import HighAccuracyLocalization, HighAccuracyLocalizationRequest
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

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.reference_frame
        goal_pose.header.stamp = rospy.get_rostime()
        goal_pose.pose = self.poses_ros[self.pose_idx]

        # same time, let mpc decide the timing
        current_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(current_pose)

        goal_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(goal_pose)

        rospy.loginfo("Publishing pose {}".format(self.pose_idx))
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
        confusor_service_name = self.get_scoped_param("confusor_update_service_name")

        self.hal_timeout = self.get_scoped_param("hal_timeout")
        self.hal_update_timeout = self.get_scoped_param("hal_update_timeout")
        self.confusor_timeout = self.get_scoped_param("confusor_timeout")

        self.hal_service_client = rospy.ServiceProxy(hal_optimization_service_name, HighAccuracyLocalization)
        self.confusor_service_client = rospy.ServiceProxy(confusor_service_name, Empty)

    def execute(self, ud):
        try:
            self.hal_service_client.wait_for_service(self.hal_timeout)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return 'Aborted'

        try:
            self.confusor_service_client.wait_for_service(self.confusor_timeout)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return 'Aborted'

        hal_req = HighAccuracyLocalizationRequest()
        hal_res = self.hal_service_client.call(hal_req)
        rospy.loginfo("Update base pose in world frame is: {}".format(hal_res.corrected_base_pose_in_world))

        # TODO(giuseppe) use the correct confusor service class and pass the hal response to it
        self.confusor_service_client.call()
        return 'Completed'


class InitializeGrinding(BaseStateRos):
    """
    Start the grinder
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.timeout = self.get_scoped_param("timeout")

    def execute(self, ud):
        rospy.logwarn("The execution of the InitializeGrinding state needs to be implemented. Sleeping for 3.0 s")
        rospy.sleep(3.0)
        return 'Completed'


class InitialPositioning(EndEffectorRocoControl):
    """
    """
    def __init__(self, ns):
        EndEffectorRocoControl.__init__(self, ns=ns)
        self.offset = self.get_scoped_param("offset")

    def execute(self, ud):
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

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.reference_frame
        goal_pose.header.stamp = rospy.get_rostime()

        # get the first pose in the task path.
        goal_pose = copy.deepcopy(self.get_context_data("grinding_path"))
        if not goal_pose:
            rospy.logwarn("Could not get the first pose from the grinding path")

        # same time, let mpc decide the timing
        current_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(current_pose)

        goal_pose.header.stamp = rospy.get_rostime()
        goal_path.poses.append(goal_pose)

        rospy.loginfo("Publishing initiating path")
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

    def execute(self, ud):
        rospy.logwarn("The execution of the MoveIntoContact state needs to be implemented. Sleeping for 3.0 s")
        rospy.sleep(3.0)
        return 'Completed'


class DoGrinding(BaseStateRos):
    """
    Does the grinding
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.timeout = self.get_scoped_param("timeout")

    def execute(self, ud):
        rospy.logwarn("The execution of the DoGrinding state needs to be implemented. Sleeping for 3.0 s")
        path = self.get_context_data("grinding_path")
        rospy.loginfo("Path has len {}".format(len(path.poses)))
        rospy.sleep(3.0)
        return 'Completed'


class AsBuiltSensing(BaseStateRos):
    """
    State description
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)

    def execute(self, ud):
        rospy.logwarn("The execution of the AsBuiltSensing state needs to be implemented. Sleeping for 3.0 s")
        rospy.sleep(3.0)
        return 'Completed'
