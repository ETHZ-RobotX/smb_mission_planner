#!/usr/bin/env python2

import rospy
import oyaml as yaml
import tf
from geometry_msgs.msg import PoseStamped
from smb_mission_planner.srv import AddMission, AddMissionResponse

class MissionRecorder():
    def __init__(self, yaml_file_path):
        self.yaml_file_path = yaml_file_path

        self.current_mission_name = ""
        self.current_goal_list = []
        self.goal_counter = 0
        self.missions_data = {}

        self.mission_service = rospy.Service('add_mission', AddMission, self.addMission)
        self.pose_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.recordPose)

        self.main()

    def addMission(self, data):
        if(data.mission_name == ""):
            rospy.logwarn("The mission name cannot be empty. Recording of mission cancelled.")
            return AddMissionResponse()
        self.current_mission_name = data.mission_name

        current_goal_list = map(str.strip, data.points_of_interest.split(','))
        for goal_name in current_goal_list:
            if(goal_name == ""):
                rospy.logwarn("Names of points of interest cannot be empty. Recording of mission cancelled.")
                return AddMissionResponse()
        self.current_goal_list = current_goal_list

        self.goal_counter = 0
        rospy.loginfo("Recording of mission '" + self.current_mission_name + "' has started.")
        rospy.loginfo("Please record the pose of the first point of interest '" + self.current_goal_list[0] + "'.")
        return AddMissionResponse()

    def recordPose(self, pose_stamped_msg):
        if(self.goal_counter < len(self.current_goal_list)):
            self.addPose(self.current_mission_name, self.current_goal_list[self.goal_counter], pose_stamped_msg)
            rospy.loginfo("The pose of the point of interest '" + self.current_goal_list[self.goal_counter] + "' has been succesfully recorded.")
            self.goal_counter += 1
            if(self.goal_counter == len(self.current_goal_list)):
                rospy.loginfo("Recording of mission '" + self.current_mission_name + "' has been successfully completed.")
            else:
                rospy.loginfo("Please record the pose of the next point of interest '" + self.current_goal_list[self.goal_counter] + "'.")

    def addPose(self, mission_name, goal_name, pose_stamped_msg):
        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        new_pose = {goal_name: {'x_m': x_m, 'y_m': y_m, 'yaw_rad': yaw_rad}}
        if(self.missions_data.get(mission_name) == None):
            self.missions_data.update({mission_name: new_pose})
        else:
            self.missions_data[mission_name].update(new_pose)

    def dump(self):
        with open(self.yaml_file_path, 'w') as file:
            yaml.dump(self.missions_data, file, default_flow_style=False)
            rospy.loginfo("Mission file succesfully dumped under: " + self.yaml_file_path)

    def main(self):
        rospy.init_node('mission_recorder_node')
        rospy.loginfo("Mission recorder ready. Waiting for '/add_mission' services.")

        # Wait for Ctrl-C to stop the application.
        rospy.spin()
        self.dump()
