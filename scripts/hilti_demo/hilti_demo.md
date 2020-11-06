### HILTI Demo

This folder contains the scripts implementing the state machine for the HILTI demo. 
A summary of the current development status of the state machine is contained in this [live document](https://docs.google.com/drawings/d/1Y1SLJT0B-n-dx-Pc8aVBVGNic1gciBG6ic9OO73MfFA/edit?usp=sharing).

The states' implementation can be found [here](../../src/smb_mission_planner/grinding_mission).
 
Each state has its own set of parameters which is defined centrally in a mission config file. 
Each state is namespaced and parameters should be set in this namespace. It is a good practice to have
the namespace recalling the state name. This comes to the cost of repeating params which are shared
among states but in turns make sure that these are explicitly defined for each state avoiding silent bugs. 

The current mission file can be found [here](../../configs/hilti_demo/mission_config.yaml).

### Run debug demo
[This script](hilti_mock_modules.py) and this [path generation node](path_generator.py) provides the necessary
components to test the state machine without all the machinery in place. They create all subscribers, 
publishers and service server which the state machine is expecting to call. The path generator creates a vertical square
path in place of the input from the building model.
 
The names specified in the [mockup modules launch file](../../launch/hilti_demo/hilti_mock_modules.launch) needs to 
coincide with the same in the [mission file](../../configs/hilti_demo/mission_config.yaml). 

To run:
`roslaunch smb_mission_planner hilti_demo.launch mock_modules:=true`.