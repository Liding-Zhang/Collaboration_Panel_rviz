# Multi-Robot Simulation and RViz GUI 
This repository contains the dependency packages `collaboration_launch` and `rviz_collabo`.
The first package is responsible for launching the multi-robot simulation with the DARKO robot cosntisting of the RB-Kairos base and the Franke Emika Panda arm attached to the base, in combination with the MiR 100 robot. 
The second package delivers a custom RViz panel for navigation and motion planning of the mentioned multi-robot simulation.

# Install before Build (dependencies)
Install the following:
```
sudo apt-get install libspdlog-dev
```

## Build
Clone this repository into the `src` folder of your workspace:
```
cd ~/path_to_workspace/src
git clone https://github.com/michilem/Collaboration_Panel.git .
```
Next enter the root of your workspace and run:
```
catkin build
```
After building the packages, make sure to source the workspace:
```
source devel/setup.bash
```

## Run the Simulation
First the robots need to be spawned into gazebo:
```
roslaunch collaboration_launch multi_robot_gazebo.launch
```
Next launch the navigation and MoveIt functionalities of the robots:
```
roslaunch collaboration_launch multi_robot_nav.launch
```
Only afterwards launch RViz with the custom GUI Panel since it assumes the robot simulation to be running.<br>
Option 1: Directly launch RViz with the Panel:
```
roslaunch rviz_collabo rviz_collabo.launch
```
**OR** <br>
Option 2: Launch RViz without the Panel first:
```
rviz -d $(rospack find rbkairos_gazebo)/rviz/rbkairos_multi.rviz
```
And start the Panel manually by clicking on `Panels` -> `Add New Panel` -> `CollaborationPanel`.
