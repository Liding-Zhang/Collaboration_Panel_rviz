# RViz Custom Panel package
This package contains the custom RViz Panel for controlling the multi-robot setup of DARKO and MiR robot. 

## Launch file and parameters
The launch file `rviz_collabo.launch` launches RViz by running:
```
roslaunch rviz_collabo rviz_collabo.launch
```
The following arguments can be set when running the command:
- `config` specifies the rviz configuration that should be loaded. The default value is a configuration tailored to the multi-robot simulation with the Panel loaded.
- `mir_prefix` describes the namespace under which the MiR robot is operated in simulation. Default value is `mir/`.
- `rbk_prefix` describes the namespace under which the DARKO robot is operated in simulation. Default value is `rbkairos/`.
- `map_frame_id` specifies the map frame id which is set to `map` for the simulation and is therefor also the default value.
- `panda_group_name` holds the name of the planning group for the Franka Emika panda arm manipulator. Default value is `panda_arm`.
- `gripper_group_name` holds the name of the planning group for the gripper attached to the panda arm manipulator. Default value is `panda_hand`.

When adjusting any of these parameters for the simulation, they should also be adjusted accordingly when launching the RViz Panel. 
