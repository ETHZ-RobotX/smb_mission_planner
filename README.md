## General Info
This repo adds mission planning functionality to the SMB stack. The implementation uses the [smach](http://wiki.ros.org/smach) library.
The library contains a collection of `State`s. Each `State` must implement the `execute` function which is run when 
a transition to this state happens.
We try to organize states in different categories and keep states as less interconnected as possible to 
avoid couplings which are hard to debug later on.

#### Library overview
- `navigation_states.py` : contains all navigation related states, as for example parsing and navigating
through a list of waypoints. Check the script for more documentation.
- `detection_states.py`: contains all detection related states. One might implement object detection using a 
service call. We already provide a detection service which is called in the execute function and this expect the 
object name, pose and success flag to be returned by the detector. 
- `manipulation_states.py`: as the name suggests, this script contains all manipulation related states. All implemented 
 states are currently based on MoveIt. More info in the states' documentation.
- `utils`: collection of utilities which are frequently used by mission states. These can be a wrapper to switch controller, 
sending moveit a specific plan or recording a waypoint-based navigation mission.

#### Constructing a State Machine
A mission plan (state machine) defines the connections (transitions) between different states. 
The mission plan is implemented as a `smach` state machine.

The proposed workflow for planning a navigation mission could be as follows:

1. Record mission data by using the `mission_recorder`
2. Create a plan for your missions in the `mission_plan`
3. Execute your missions by using the `mission_planner`

Each of these steps is explained in detail in the upcoming sections.


## Installation
(Could be added to the dependencies at a later stage.)

- Clone this repo into your workspace
- Install `smach` from [here](http://wiki.ros.org/smach) (a state machine library for python)
- Install `smach_ros` from [here](http://wiki.ros.org/smach_ros) (ROS compatibility for `smach`, e.g. for the viewer in rqt)
- Install `smach_viewer` 
- Install `oyaml` with `pip install oyaml` (enables python2 compatibility with ordered dicts for `yaml` file dump)
- (Buid the package with `catkin build smb_mission_planner`)

## Tutorials
A tutorial on how to record and execute a waypoint-based navigation mission is available [here](docs/navigation_mission_tutorial.md)
## Examples

#### Combined mission
The robot has a manipulator which uses to scan different viewpoint after a base goal has been reached

- Terminal #1: `roslaunch smb_man_sim sim.launch`
- Terminal #2: `roslaunch smb_mission_planner detection_mission_smb_kinova.launch`

#### Navigation mission
The robot moves to different goal and reaches each goal through a set of predefined waypoints

- Terminal #1: `roslaunch smb_man_sim sim.launch`
- Terminal #3: `roslaunch smb_mission_planner navigation_mission fake_execution:=false`


#### TODO 
- More detection states
- More manipulation states




