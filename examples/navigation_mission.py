import rospy
import smach
import smach_ros
from smb_mission_planner.navigation_states import WaypointNavigation


if __name__ == "__main__":
    rospy.init_node('mission_planner_node')

    # Parse params
    mission_file = rospy.get_param("mission_file")
    move_base_topic = rospy.get_param("move_base_topic")
    odometry_topic = rospy.get_param("odometry_topic")

    state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
    mission_data = WaypointNavigation.read_missions_data(mission_file)
    with state_machine:
        smach.StateMachine.add('Mission 1', WaypointNavigation(mission_data['check_fire_hazard'],
                                                               waypoint_pose_topic=move_base_topic,
                                                               base_pose_topic=odometry_topic),
                               transitions={'Completed': 'Mission 2',
                                            'Aborted': 'Failure',
                                            'Next Waypoint': 'Mission 1'})

        smach.StateMachine.add('Mission 2', WaypointNavigation(mission_data['gather_fruits'],
                                                               waypoint_pose_topic=move_base_topic,
                                                               base_pose_topic=odometry_topic),
                               transitions={'Completed': 'Success',
                                            'Aborted': 'Mission 3',
                                            'Next Waypoint': 'Mission 2'})

        smach.StateMachine.add('Mission 3', WaypointNavigation(mission_data['gather_vegetables'],
                                                               waypoint_pose_topic=move_base_topic,
                                                               base_pose_topic=odometry_topic),
                               transitions={'Completed': 'Success',
                                            'Aborted': ' Failure',
                                            'Next Waypoint': 'Mission 3'})

    # Create and start the introspection server
    introspection_server = smach_ros.IntrospectionServer('mission_server', state_machine, '/mission_planner')
    introspection_server.start()

    # Execute state machine.
    outcome = state_machine.execute()
    rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

    # Wait for ctrl-c to stop the application
    introspection_server.stop()
