#!/usr/bin/env python2

import smach
from smb_mission_planner.utils import MoveItPlanner

"""
Here define all the navigation related states
"""


class MoveItCartesianPoseReaching(smach.State):
    def __init__(self, ):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.planner = MoveItPlanner()

    def execute(self, ud):
        self.planner.reach_cartesian_pose() # TODO 

