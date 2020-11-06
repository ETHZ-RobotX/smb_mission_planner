import numpy as np

import rospy
from std_srvs.srv import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from smb_mission_planner.base_state_ros import BaseStateRos
from smb_mission_planner.navigation_states import SingleNavGoalState
from smb_mission_planner.utils.navigation_utils import nav_goal_from_path
from smb_mission_planner.utils import rocoma_utils


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


class ArmPosesVisitor(BaseStateRos):
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
        BaseStateRos.__init__(self, outcomes=['PoseReached', 'Aborted', 'NoMorePoses'], ns=ns)
        self.timeout = self.get_scoped_param("timeout")
        self.controller = self.get_scoped_param("roco_controller")
        self.controller_manager_namespace = self.get_scoped_param("controller_manager_namespace")
        self.frame = self.get_scoped_param("frame_id")
        self.poses = self.get_scoped_param("poses")
        self.pose_idx = 0
        self.num_poses = len(self.poses)

        self.ok = self.check_poses()

        pose_topic_name = self.get_scoped_param("pose_topic_name")
        self.pose_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)

    def check_poses(self):
        for pose in self.poses:
            if 't' not in pose.keys() or 'q' not in pose.keys():
                rospy.logerr("poses param is ill-formed")
                return False

            if len(pose['t']) != 3 or len(pose['q']) != 4:
                rospy.logerr("poses param is ill-formed")
                return False

            q_arr = np.array(pose['q'])
            norm = np.linalg.norm(q_arr)
            if norm > 1.01 or norm < 0.99:
                rospy.logerr("pose quaternion is not normalized")
                return False
        return True

    def execute(self, ud):
        if not self.ok:
            rospy.logerr("The poses list is ill-formed, aborting...")
            return 'Aborted'

        success = rocoma_utils.switch_roco_controller(self.controller,
                                                      ns=self.controller_manager_namespace)
        if not success:
            rospy.logerr("ArmPoseVisitor failed")
            return 'Aborted'

        if self.pose_idx >= self.num_poses:
            rospy.loginfo("No more target poses!")
            return 'NoMorePoses'

        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame
        new_pose.header.stamp = rospy.get_rostime()
        new_pose.pose.position.x = self.poses[self.pose_idx]['t'][0]
        new_pose.pose.position.y = self.poses[self.pose_idx]['t'][1]
        new_pose.pose.position.z = self.poses[self.pose_idx]['t'][2]
        new_pose.pose.orientation.x = self.poses[self.pose_idx]['q'][0]
        new_pose.pose.orientation.y = self.poses[self.pose_idx]['q'][1]
        new_pose.pose.orientation.z = self.poses[self.pose_idx]['q'][2]
        new_pose.pose.orientation.w = self.poses[self.pose_idx]['q'][3]

        rospy.loginfo("Publishing pose {}".format(self.pose_idx))
        self.pose_publisher.publish(new_pose)

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
    Triggers the HAL 0optimization routine and send the received pose update to confusor for the state
    estimation update
    """
    def __init__(self, ns):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        hal_optimization_service_name = self.get_scoped_param("hal_optimization_service_name")
        update_topic_name = self.get_scoped_param("hal_update_topic_name")
        confusor_service_name = self.get_scoped_param("confusor_update_service_name")

        self.hal_timeout = self.get_scoped_param("hal_timeout")
        self.hal_update_timeout = self.get_scoped_param("hal_update_timeout")
        self.confusor_timeout = self.get_scoped_param("confusor_timeout")

        self.update_subscriber = rospy.Subscriber(update_topic_name, PoseStamped, self.update_cb, queue_size=1)
        self.hal_service_client = rospy.ServiceProxy(hal_optimization_service_name, Empty)
        self.confusor_service_client = rospy.ServiceProxy(confusor_service_name, Empty)
        self.confusor_updated = False

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

        # TODO(giuseppe) better to receive the update in the answer
        self.hal_service_client.call()

        elapsed = 0
        while elapsed < self.hal_update_timeout:
            if self.confusor_updated:
                return 'Completed'
            else:
                rospy.sleep(0.1)
                elapsed += 0.1

        rospy.logerr("Failed to update confusor")
        return 'Aborted'

    def update_cb(self, update):
        rospy.loginfo("Received new update from HAL routine, sending to Confusor")
        rospy.logwarn("Need to implement the actual confusor service")
        self.confusor_service_client.call()
        self.confusor_updated = True


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
