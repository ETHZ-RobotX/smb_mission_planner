#!/usr/bin/env python2

import rospy
import collections
import oyaml as yaml
import tf
from geometry_msgs.msg import PoseStamped
from smb_mission_planner.srv import AddMission, AddMissionResponse
from smb_mission_planner.srv import DeleteMission, DeleteMissionResponse
from smb_mission_planner.srv import DeleteGoal, DeleteGoalResponse


class MissionRecorder():
    def __init__(self, yaml_file_path, goal_topic_name):
        self.yaml_file_path = yaml_file_path
        self.goal_topic_name = goal_topic_name

        self.current_mission_name = ""
        self.current_goal_list = []
        self.goal_counter = 0
        self.missions_data = collections.OrderedDict()

        self.add_mission_service = rospy.Service('add_mission', AddMission, self.addMission)
        self.delete_mission_service = rospy.Service('delete_mission', DeleteMission, self.deleteMission)
        self.delete_goal_service = rospy.Service('delete_goal', DeleteGoal, self.deletePose)
        self.add_pose_subscriber = rospy.Subscriber(self.goal_topic_name, PoseStamped, self.recordPose)

        self.main()

    def addMission(self, data):
        if(data.mission_name == ""):
            rospy.logwarn("The mission name cannot be empty. Recording of mission cancelled.")
            return AddMissionResponse()
        self.current_mission_name = data.mission_name

        current_goal_list = list(map(str.strip, data.goal_names.split(',')))
        for goal_name in current_goal_list:
            if(goal_name == ""):
                rospy.logwarn("Goal names cannot be empty. Recording of mission cancelled.")
                return AddMissionResponse()
        self.current_goal_list = current_goal_list

        self.goal_counter = 0
        rospy.loginfo("Recording of mission '" + self.current_mission_name + "' has started.")
        rospy.loginfo("Please record the pose of the goal '" + self.current_goal_list[0] + "'.")
        return AddMissionResponse()

    def deleteMission(self, data):
        if(data.mission_name == ""):
            rospy.logwarn("The mission name cannot be empty. Deleting of mission cancelled.")
            return DeleteMissionResponse()
        try:
            del self.missions_data[data.mission_name]
            rospy.loginfo("Mission '" + self.current_mission_name + "' deleted.")
            return DeleteMissionResponse()
        except:
            rospy.logwarn("The mission '" + data.mission_name + "'does not exist. Deleting of mission cancelled.")
            return DeleteMissionResponse()

    def recordPose(self, pose_stamped_msg):
        if(self.goal_counter < len(self.current_goal_list)):
            self.addPose(self.current_mission_name, self.current_goal_list[self.goal_counter], pose_stamped_msg)
            rospy.loginfo("The pose of the goal '" + self.current_goal_list[self.goal_counter] + "' has been succesfully recorded.")
            self.goal_counter += 1
            if(self.goal_counter == len(self.current_goal_list)):
                rospy.loginfo("Recording of mission '" + self.current_mission_name + "' has been successfully completed.")
            else:
                rospy.loginfo("Please record the pose of the next goal '" + self.current_goal_list[self.goal_counter] + "'.")

    def addPose(self, mission_name, goal_name, pose_stamped_msg):
        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        new_pose = collections.OrderedDict({goal_name: {'x_m': x_m, 'y_m': y_m, 'yaw_rad': yaw_rad}})
        if(self.missions_data.get(mission_name) == None):
            self.missions_data.update({mission_name: new_pose})
        else:
            self.missions_data[mission_name].update(new_pose)

    def deletePose(self, data):
        if(data.mission_name == ""):
            rospy.logwarn("The mission name cannot be empty. Deleting of goal cancelled.")
            return DeleteGoalResponse()
        if(data.goal_name == ""):
            rospy.logwarn("The goal name cannot be empty. Deleting of goal cancelled.")
            return DeleteGoalResponse()
        try:
            del self.missions_data[data.mission_name][data.goal_name]
            rospy.loginfo("Goal '" + data.goal_name + "' in mission '" + data.mission_name + "' deleted.")
            return DeleteGoalResponse()
        except:
            rospy.logwarn("The goal '" + data.mission_name + "/" + data.goal_name + "' does not exist.")
            return DeleteGoalResponse()

    def dump(self):
        with open(self.yaml_file_path, 'w') as file:
            yaml.dump(self.missions_data, file, default_flow_style=False)
            rospy.loginfo("Mission file succesfully dumped under: " + self.yaml_file_path)

    def main(self):
        rospy.init_node('mission_recorder_node')
        rospy.loginfo("Mission recorder ready.")
        rospy.loginfo("Waiting for '/add_mission' services.")

        # Wait for Ctrl-C to stop the application.
        rospy.spin()
        self.dump()
