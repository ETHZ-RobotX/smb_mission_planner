#!/usr/bin/env python2

import argparse
from smb_mission_planner.mission_planner import MissionPlanner

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Tool for planning missions.')
    parser.add_argument('config_file_path', type=str, help='Path of the config file.')
    # Ignore arguments sent by roslaunch.
    parser.add_argument('__name:', help=argparse.SUPPRESS, nargs='?')
    parser.add_argument('__log:', help=argparse.SUPPRESS, nargs='?')
    args = parser.parse_args()

    mission_planner = MissionPlanner(args.config_file_path)
