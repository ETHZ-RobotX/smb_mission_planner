import smach
import smach_ros
import mission_planner


def createMissionPlan(missions_data):
    state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])

    with state_machine:
        smach.StateMachine.add('Mission 1', mission_planner.Mission(missions_data['fire_hazard']),
        transitions={'Completed':'Mission 2', 'Aborted':'Failure', 'Next Goal':'Mission 1'})

        smach.StateMachine.add('Mission 2', mission_planner.Mission(missions_data['gather_fruits']),
        transitions={'Completed':'Success', 'Aborted':'Mission 3', 'Next Goal':'Mission 2'})

        smach.StateMachine.add('Mission 3', mission_planner.Mission(missions_data['gather_vegetables']),
        transitions={'Completed':'Success', 'Aborted':'Failure', 'Next Goal':'Mission 3'})

    return state_machine
