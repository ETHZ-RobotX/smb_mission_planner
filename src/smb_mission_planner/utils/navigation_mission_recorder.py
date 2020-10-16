#!/usr/bin/env python2

import rospy
import collections
import oyaml as yaml
import tf
from geometry_msgs.msg import PoseStamped
from smb_mission_planner.srv import RecordMission, RecordMissionResponse
from smb_mission_planner.srv import RemoveMission, RemoveMissionResponse
from smb_mission_planner.srv import RemoveWaypoint, RemoveWaypointResponse
from smb_mission_planner.srv import ToggleFileDump, ToggleFileDumpResponse
from smb_mission_planner.srv import RecordBasePose, RecordBasePoseResponse


class NavigationMissionRecorder(object):
    def __init__(self, file_path, waypoint_topic, base_pose_topic):
        self.yaml_file_path = file_path
        self.waypoint_topic_name = waypoint_topic
        self.base_pose_topic_name = base_pose_topic

        self.current_mission_name = ""
        self.current_waypoint_list = []
        self.waypoint_counter = 0
        self.missions_data = collections.OrderedDict()
        self.file_dump_on = True

        self.waypoint_pose_subscriber = rospy.Subscriber(self.waypoint_topic_name, PoseStamped, self.waypoint_cb)
        self.base_pose_subscriber = rospy.Subscriber(self.base_pose_topic_name, PoseStamped, self.base_pose_cb)
        self.record_mission_service = rospy.Service('record_mission', RecordMission, self.record_mission)
        self.remove_mission_service = rospy.Service('remove_mission', RemoveMission, self.remove_mission)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.remove_waypoint)
        self.record_base_pose_service = rospy.Service('record_base_pose', RecordBasePose, self.record_base_pose)
        self.toggle_file_dump_service = rospy.Service('toggle_file_dump', ToggleFileDump, self.toggle_file_dump)

        self.base_pose_msg = PoseStamped()
        self.print_info()

    def print_info(self):
        info = """Waypoint recorder:
writing to file: {}
listening to waypoint on topic: {}
listening to base pose on topic: {} 
""".format(self.yaml_file_path, self.waypoint_topic_name, self.base_pose_topic_name)
        rospy.loginfo(info)

    def record_mission(self, data):
        if data.mission_name == "":
            rospy.logwarn("The mission name cannot be empty. Recording of mission cancelled.")
            return RecordMissionResponse()
        self.current_mission_name = data.mission_name

        current_waypoint_list = list(map(str.strip, data.waypoint_names.split(',')))
        for waypoint_name in current_waypoint_list:
            if waypoint_name == "":
                rospy.logwarn("Waypoint names cannot be empty. Recording of mission cancelled.")
                return RecordMissionResponse()
        self.current_waypoint_list = current_waypoint_list

        self.waypoint_counter = 0
        rospy.loginfo("Recording of mission '" + self.current_mission_name + "' has started.")
        rospy.loginfo("Please record the pose of the waypoint '" + self.current_waypoint_list[0] + "'.")
        return RecordMissionResponse()

    def remove_mission(self, data):
        if data.mission_name == "":
            rospy.logwarn("The mission name cannot be empty. Deleting of mission cancWelled.")
            return RemoveMissionResponse()
        try:
            del self.missions_data[data.mission_name]
            rospy.loginfo("Mission '" + self.current_mission_name + "' removed.")
            return RemoveMissionResponse()
        except:
            rospy.logwarn("The mission '" + data.mission_name + "'does not exist. Deleting of mission cancelled.")
            return RemoveMissionResponse()

    def waypoint_cb(self, msg):
        assert isinstance(msg, PoseStamped)
        if self.waypoint_counter < len(self.current_waypoint_list):
            self.add_waypoint(self.current_mission_name, self.current_waypoint_list[self.waypoint_counter], msg)
            rospy.loginfo("The pose of the waypoint '" +
                          self.current_waypoint_list[self.waypoint_counter] +
                          "' has been successfully recorded.")
            self.waypoint_counter += 1

            if self.waypoint_counter == len(self.current_waypoint_list):
                rospy.loginfo("Recording of mission '" + self.current_mission_name +
                              "' has been successfully completed.")
            else:
                rospy.loginfo("Please record the pose of the next waypoint '" +
                              self.current_waypoint_list[self.waypoint_counter] + "'.")

    def add_waypoint(self, mission_name, waypoint_name, pose_stamped_msg):
        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        new_waypoint = collections.OrderedDict({waypoint_name: {'x_m': x_m, 'y_m': y_m, 'yaw_rad': yaw_rad}})
        if not self.missions_data.get(mission_name):
            self.missions_data.update({mission_name: new_waypoint})
        else:
            self.missions_data[mission_name].update(new_waypoint)

    def remove_waypoint(self, data):
        if data.mission_name == "":
            rospy.logwarn("The mission name cannot be empty. Deleting of waypoint cancelled.")
            return RemoveWaypointResponse()
        if data.waypoint_name == "" :
            rospy.logwarn("The waypoint name cannot be empty. Deleting of waypoint cancelled.")
            return RemoveWaypointResponse()
        try:
            del self.missions_data[data.mission_name][data.waypoint_name]
            rospy.loginfo("Waypoint '" + data.waypoint_name + "' in mission '" + data.mission_name + "' removed.")
            return RemoveWaypointResponse()
        except:
            rospy.logwarn("The waypoint '" + data.mission_name + "/" + data.waypoint_name + "' does not exist.")
            return RemoveWaypointResponse()

    def base_pose_cb(self, msg):
        assert isinstance(msg, PoseStamped)
        self.base_pose_msg = msg

    def record_base_pose(self):
        self.waypoint_cb(self.base_pose_msg)
        return RecordBasePoseResponse()

    def toggle_file_dump(self, data):
        self.file_dump_on = data.file_dump_on
        if self.file_dump_on:
            rospy.loginfo("Dumping to file is currently on.")
        else:
            rospy.logwarn("Dumping to file is currently off.")
        return ToggleFileDumpResponse()

    def dump(self):
        if self.file_dump_on:
            with open(self.yaml_file_path, 'w+') as stream:
                yaml.dump(self.missions_data, stream, default_flow_style=False)
                rospy.loginfo("Mission file successfully dumped under: " + self.yaml_file_path)

    def run(self):
        rospy.init_node('mission_recorder_node')
        rospy.loginfo("Mission recorder ready.")
        rospy.loginfo("Waiting for '/record_mission' services.")
        self.print_info()

        # Wait for Ctrl-C to stop the application.
        rospy.spin()
        self.dump()
