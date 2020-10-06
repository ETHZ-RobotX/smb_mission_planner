## General Info

This repo adds mission planning functionality to the stack as follows:

1. Record missions by using the `mission_recorder`
2. Create a plan for your missions in the `mission_plan`
3. Execute your missions by using the `mission_planner`


## Installation

- Clone this repo into your workspace
- Install `smach` from [here](http://wiki.ros.org/smach) (a state machine library for python)
- Install `smach_ros` from [here](http://wiki.ros.org/smach_ros) (ROS compatibility for `smach`, e.g. for the viewer in rqt.)
- Install `oyaml` with `pip install oyaml` (enables python2 compatibility with ordered dicts for yaml file dump)







## Record missions

As it is fairly tedious to input poses manually for the mission goals, the `mission_recorder` helps you out.
It generates a `yaml` file with all the goal poses you recorded, grouped by mission.
You can launch it with 
```
roslaunch smb_mission_planner mission_recorder.launch
```
which starts the node. 
You then can give recording instructions with ros services. 
To record a mission, call
`rosservice call /record_mission {"mission_name","goal_1_name, goal_2_name, ..."}`
where you can use your own `mission_name` and `goal_names`.
The number of goals can be selected arbitrarily, just add more to the list.
After you sent the `/record_mission` service, instructions will appear in the command window where you launched the node.
You can now input the poses of the goals of the current mission one by one.
This can be done 

- in `rviz` by clicking `2D Nav Goal` and visually placing the pose on your map. 
- by sending the desired pose in the topic `/move_base_simple/goal`.
- in `rviz` by sending a goal with the `smb_path_planner` widget.

After having recorded all your missions, stop the node with `Ctrl-C`. 
All your recorded missions will be dumped to the `yaml` file.
Of course, you can also *manually edit* the generated yaml file, to combine different recording sessions, add or edit goals manually, etc.

### Advanced Features

#### Delete Missions
Delete missions while the node is running with
```
rosservice call /delete_mission "mission_name"
```

#### Delete Goals
Delete missions while the node is running with
```
rosservice call /delete_goal {"mission_name","goal_name"}
```

#### Specify file for file dump
You can use a `roslaunch` argument to specify a filepath for the output file, e.g.
```
roslaunch smb_mission_planner mission_recorder.launch yaml_file_path:~/smb_catkin_ws/src/smb_mission_planner/config/my_amazing_config.yaml
```

#### Prevent file dump
Stop file dump with
```
rosservice call /toggle_file_dump "False"
```
or reenable it with "True"


#### Choose your own input topic for recording
You can use a `roslaunch` argument to specify the input topic for the recording
```
roslaunch smb_mission_planner mission_recorder.launch goal_pose_topic:=/move_base_simple/goal
```





## Mission planning

You can combine your recorded missions to a mission plan by connecting them to each other in the `mission_plan.py` to specify a specific behaviour.
The `mission_plan.py` should be modified by you to add more missions and connect them accordingly.
Here, a `smach` state machine is built up.
To learn more about it, visit the [tutorials](http://wiki.ros.org/smach/Tutorials).
Make sure to assign to each mission its respective mission data, i.e. its recorded information of the yaml file.



## Executing you mission plan




## Where to go from here








## Example tutorial



## Common pitfalls

- It is easy to forget to change the mission names in the `mission_plan.py` when recording new missions.
- The rosservice call must be in the form `{"mission_name","goal_1_name, goal_2_name, ..."}` without a space after the separating comma.









