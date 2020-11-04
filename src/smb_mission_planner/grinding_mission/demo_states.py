import rospy
from nav_msgs.msg import Path
from smb_mission_planner.base_state_ros import BaseStateRos
from smb_mission_planner.navigation_states import SingleNavGoalState
from smb_mission_planner.utils.navigation_utils import nav_goal_from_path


class MissionStarter(BaseStateRos):
    def __init__(self, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_subscriber = rospy.Subscriber(self.path_topic_name, Path)
        self.timeout = self.get_scoped_param("timeout")
        self.data_received = False

    def path_callback(self, msg):
        self.set_context_data("grinding_path", msg)
        self.data_received = True

    def execute(self, ud):
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
    def __init__(self, ns=""):
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


class HighAccuracyLocalization(BaseStateRos):
    def __init__(self, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.timeout = self.get_scoped_param("timeout")

    def execute(self, ud):
        rospy.logwarn("The execution of the HAL state needs to be implemented. Sleeping for 3.0 s")
        rospy.sleep(3.0)
        return 'Completed'


class InitializeGrinding(BaseStateRos):
    def __init__(self, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.timeout = self.get_scoped_param("timeout")

    def execute(self, ud):
        rospy.logwarn("The execution of the InitializeGrinding state needs to be implemented. Sleeping for 3.0 s")
        rospy.sleep(3.0)
        return 'Completed'


class DoGrinding(BaseStateRos):
    def __init__(self, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)
        self.timeout = self.get_scoped_param("timeout")

    def execute(self, ud):
        rospy.logwarn("The execution of the DoGrinding state needs to be implemented. Sleeping for 3.0 s")
        path = self.get_context_data("grinding_path")
        rospy.loginfo("Path has len {}".format(len(path.poses)))
        rospy.sleep(3.0)
        return 'Completed'
