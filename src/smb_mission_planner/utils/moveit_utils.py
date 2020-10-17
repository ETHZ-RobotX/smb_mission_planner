#!/usr/bin/env python

from moveit_msgs.msg import Constraints, OrientationConstraint

import os
import sys
import rospy
import moveit_commander
import moveit_msgs.msg


class MoveItPlanner(object):
    """ExampleMoveItTrajectories"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.fake_execution = rospy.get_param("/moveit_planner/fake_execution", False)
        rospy.loginfo("MoveItPlanner fake_execution: {}".format(self.fake_execution))

        if not self.fake_execution:
            try:
                self.is_gripper_present = rospy.get_param("/moveit_planner/is_gripper_present", False)
                if self.is_gripper_present:
                    gripper_joint_names = rospy.get_param("/moveit_planner/gripper_joint_names", [])
                    self.gripper_joint_name = gripper_joint_names[0]
                else:
                    self.gripper_joint_name = ""
                self.degrees_of_freedom = rospy.get_param("/moveit_planner/degrees_of_freedom", 7)

                # Create the MoveItInterface necessary objects
                self.arm_group_name = rospy.get_param("/moveit_planner/arm_group_name")
                self.description_name = rospy.get_param("/moveit_planner/description_name")
                self.robot = moveit_commander.RobotCommander(self.description_name)

                self.namespace = rospy.get_param("/moveit_planner/namespace")
                self.scene = moveit_commander.PlanningSceneInterface(ns=self.namespace)

                self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name,
                                                                     robot_description=self.description_name,
                                                                     ns=self.namespace)
                self.display_trajectory_publisher = rospy.Publisher(
                    os.path.join(self.namespace + 'move_group/display_planned_path'),
                    moveit_msgs.msg.DisplayTrajectory,
                    queue_size=20)

                if self.is_gripper_present:
                    gripper_group_name = "gripper"
                    self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=self.namespace)

                self.planning_frame = self.arm_group.get_planning_frame()
                rospy.loginfo("Planning in the reference frame: {}".format(self.planning_frame))
                rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
            except Exception as e:
                print(e)
                self.is_init_success = False
            else:
                self.is_init_success = True

    def reach_named_position(self, target):
        if self.fake_execution:
            rospy.sleep(1.0)
            return True

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        self.arm_group.set_named_target(target)
        # Plan the trajectory
        planned_path1 = self.arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return self.arm_group.execute(planned_path1, wait=True)

    def reach_joint_angles(self, joint_positions, tolerance):
        if self.fake_execution:
            rospy.sleep(1.0)
            return True

        success = True

        # Get the current joint positions
        joint_positions_current = self.arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions_current:
            rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if len(joint_positions) != self.degrees_of_freedom:
            rospy.logerr("Desired joint positions have size {} != dof (={})".format(len(joint_positions),
                                                                                    self.degrees_of_freedom))
            return False

        self.arm_group.set_joint_value_target(joint_positions)
        success &= self.arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = self.arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions:
            rospy.loginfo(p)
        return success

    def get_cartesian_pose(self):

        # Get the current pose and display it
        pose = self.arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        if self.fake_execution:
            rospy.sleep(1.0)
            return True

        arm_group = self.arm_group

        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

    def reach_gripper_position(self, relative_position):
        if self.fake_execution:
            rospy.sleep(1.0)
            return True

        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(
                relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos,
                True)
            return val
        except:
            return False
