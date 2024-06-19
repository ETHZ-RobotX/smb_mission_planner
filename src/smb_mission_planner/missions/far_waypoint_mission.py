#!/usr/bin/env python
import smach
import rospy
import tf
import smach
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool

class FARWaypointMission(smach.State):
    def __init__(self, mission_data, reference_frame):
        smach.State.__init__(
            self, outcomes=['Completed', 'Aborted', 'Next Waypoint'])

        # Get parameters
        # Frames
        self.reference_frame = reference_frame
        self.robot_frame = "base_link"

        self.mission_data = mission_data
        self.waypoint_idx = 0
        self.waypoint_name = ""
        self.next_waypoint = False
        self.current_waypoint_reached = False
        self.max_time_out = 200
        self.status_check_rate = rospy.Rate(10)

        # tf
        self.listener = tf.TransformListener()

        # way point publisher to far
        self.waypoint_pub = rospy.Publisher("/goal_point", PointStamped, queue_size=1)
        self.planner_status_sub = rospy.Subscriber("/far_reach_goal_status", Bool, self.planner_status_callback)
        rospy.loginfo("FARWaypointMission initialization finished")

    def execute(self, userdata):
        if(self.waypoint_idx >= len(self.mission_data.keys())):
            rospy.loginfo("No more waypoints left in current mission.")
            self.waypoint_idx = 0
            return 'Completed'

        self.waypoint_name = list(self.mission_data.keys())[
            self.waypoint_idx]
        current_waypoint = self.mission_data[self.waypoint_name]

        self.setWaypoint(
            current_waypoint['x_m'], current_waypoint['y_m'])
        rospy.loginfo("Waypoint set: '" + self.waypoint_name + "'.")

        self.next_waypoint = self.wait_and_check_status()

        self.current_waypoint_reached = False
        if self.next_waypoint:
            rospy.loginfo("Waypoint '" + self.waypoint_name +
                              "' reached. Loading next waypoint...")
            self.waypoint_idx += 1
            return 'Next Waypoint'
        else:
            # stop FAR planner by sending the current robot_pose
            (trans, rot) = self.get_current_robot_pose()
            self.setWaypoint(trans[0], trans[1])
            rospy.logwarn(
                "Waypoint of mission unreachable. Aborting current mission.")
            self.waypoint_idx = 0.
            return 'Aborted'

    def wait_and_check_status(self):
        '''
        Wait and check the goal_reached status,
        once reached return true
        if not reached after time out, return false
        '''
        checking_endtime = rospy.Time.now() + rospy.Duration(self.max_time_out)

        while rospy.Time.now() < checking_endtime:
            rospy.loginfo("waiting for FAR reaching the goal ... ... ") 
            if self.current_waypoint_reached:
                return True
            self.status_check_rate.sleep()
        
        return False

    def setWaypoint(self, x_m, y_m):
        goal = PointStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.reference_frame
        goal.point.x = x_m
        goal.point.y = y_m
        goal.point.z = 0
        
        rospy.loginfo("publishing to the /goal_point topic")
        goal_publish_endtime = rospy.Time.now() + rospy.Duration(1)

        while rospy.Time.now() < goal_publish_endtime:
            self.waypoint_pub.publish(goal)
            self.status_check_rate.sleep()

    def planner_status_callback(self, goal_reached_status):
        if goal_reached_status.data:
            self.current_waypoint_reached = True
        else:
            self.current_waypoint_reached = False

    def get_current_robot_pose(self):

        try:
            # Listen for the transformation
            (trans, rot) = self.listener.lookupTransform(self.reference_frame, self.robot_frame, rospy.Time(0))
            return (trans, rot)
        except (tf.LookupException):
            rospy.logerr("Cannot get the current robot pose " + self.robot_frame + " to " + self.reference_frame + ", therefore not stopping FAR planner")
        