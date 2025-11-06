# Multi-Robot Simulation and RViz GUI Design
This repository contains the dependency packages `collaboration_launch` and `rviz_collabo`.
The first package is responsible for launching the multi-robot simulation with the DARKO robot, consisting of the RB-Kairos base and the Franke Emika Panda arm attached to the base, in combination with the MiR 100 robot. 
The second package provides a custom RViz panel for navigation and motion planning in the aforementioned multi-robot simulation.

# Install before Build (dependencies)
Install the following:
```
sudo apt-get install libspdlog-dev
```
# Install ROS packages (dependencies with python3 and pip)
```
  #!/bin/bash
  sudo apt install -y python3-pip
  python3 -m pip install scipy
  python3 -m pip install roboticstoolbox-python
  sudo apt-get install -y ros-noetic-navigation
  sudo apt-get install -y ros-noetic-object-recognition-msgs
  sudo apt-get install -y ros-noetic-graph-msgs
  sudo apt-get install -y ros-noetic-ruckig
  sudo apt-get install -y ros-noetic-twist-mux
  sudo apt-get install -y ros-noetic-pybind11-catkin
  sudo apt-get install -y ros-noetic-robot-localization
  sudo apt-get install -y ros-noetic-mavros-msgs
  sudo apt-get install -y ros-noetic-gmapping
  sudo apt-get install -y ros-noetic-joint-trajectory-controller
  sudo apt-get install -y ros-noetic-warehouse-ros
  sudo apt-get install -y ros-noetic-eigenpy
  sudo apt-get install -y ros-noetic-rosparam-shortcuts
  sudo apt-get install -y ros-noetic-octomap-msgs
  sudo apt-get install -y ros-noetic-eigen-stl-containers
  sudo apt-get install -y ros-noetic-random-numbers
  sudo apt-get install -y ros-noetic-velocity-controllers
  
  echo "ROS packages installation completed."
```
## Build
Clone this repository into the `src` folder of your workspace:
```
cd ~/path_to_workspace/src
git clone https://github.com/Liding-Zhang/Collaboration_Panel_rviz.git
```
Next, enter the root of your workspace and run:
```
catkin build
```
After building the packages, make sure to source the workspace:
```
source devel/setup.bash
```

## Run the Simulation
First, the robots need to be spawned into Gazebo:
```
roslaunch collaboration_launch multi_robot_gazebo.launch
```
Next, launch the navigation and MoveIt functionalities of the robots:
```
roslaunch collaboration_launch multi_robot_nav.launch
```
Only launch RViz with the custom GUI panel afterwards, as it assumes the robot simulation is already running.<br>
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
