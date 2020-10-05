#!/usr/bin/env python2

import argparse
from smb_mission_planner.mission_recorder import MissionRecorder


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Tool for recording missions.')
    parser.add_argument('config_file_path', type=str, help='Path of the config file.')
    parser.add_argument('goal_topic_name', type=str, help='Topic name for recording poses of goals.')

    # Ignore arguments sent by roslaunch.
    parser.add_argument('__name:', help=argparse.SUPPRESS, nargs='?')
    parser.add_argument('__log:', help=argparse.SUPPRESS, nargs='?')
    args = parser.parse_args()

    mission_recorder = MissionRecorder(args.config_file_path, args.goal_topic_name)
