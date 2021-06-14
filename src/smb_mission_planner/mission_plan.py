#!/usr/bin/env python

import smach
import mission_planner


def createMissionPlan(missions_data):
    state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])

    with state_machine:
        smach.StateMachine.add('Mission 1', mission_planner.DefaultMission(missions_data['check_fire_hazard']),
        transitions={'Completed':'Mission 2', 'Aborted':'Failure', 'Next Waypoint':'Mission 1'})

        smach.StateMachine.add('Mission 2', mission_planner.DefaultMission(missions_data['gather_fruits']),
        transitions={'Completed':'Success', 'Aborted':'Mission 3', 'Next Waypoint':'Mission 2'})

        smach.StateMachine.add('Mission 3', mission_planner.DefaultMission(missions_data['gather_vegetables']),
        transitions={'Completed':'Success', 'Aborted':'Failure', 'Next Waypoint':'Mission 3'})

    return state_machine
