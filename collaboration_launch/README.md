# Multi-Robot Simulation
This package contains the launch file to correctly launch and setup the multi-robot simulation.
The first launch file is `multi_robot_gazebo.launch`, followed by `multi_robot_nav.launch` for navigation and MoveIt setup.

## 1.`multi_robot_gazebo.launch`:
```
roslaunch collaboration_launch multi_robot_gazebo.launch
```
This launch file is designed to set up a multi-robot simulation environment in Gazebo, involving the DARKO and MiR robots.

---

### Important Arguments:
- **`default_arm_model`**: (`default="panda"`) Specifies the arm model to be used (default is the Franka Emika Panda arm).
- **`default_gripper_model`**: (`default="hand"`) Specifies the gripper model (default is the hand gripper).
- **`default_xacro`**: (`default="versions/rbkairos_$(arg default_arm_model)_$(arg default_gripper_model).urdf.xacro"`) Specifies the path to the robot description file (`.xacro`).
- **`gazebo_world`**: (`default="$(find rbkairos_gazebo)/worlds/office.world"`) The Gazebo world file used for simulation.

### DARKO Robot Arguments:
- **`id_rbkairos`**: (`default="rbkairos"`) Defines the robot prefix and namespace.
- **`x_init_pose_rbkairos`, `y_init_pose_rbkairos`, `z_init_pose_rbkairos`, `yaw_init_pose_rbkairos`**: Initial pose values for the DARKO robot in the Gazebo world.
- **`xacro_rbkairos`**: Specifies the URDF `.xacro` file for the DARKO robot.

---

### Included Launch Files:

1. **RB-Kairos Bringup**:
    - **File**: `rbkairos_bringup.launch`
    - **Arguments**:
      - `id_robot`: Prefix & namespace for the DARKO robot. Default: **`id_rbkairos`**
      - `x_init_pose`, `y_init_pose`, `z_init_pose`, `yaw_init_pose`: Initial position and orientation.
      - `xacro_robot`: URDF `.xacro` file for DARKO. Default: **`default_xacro`**

2. **MiR Robot Bringup**:
    - **File**: `mir_bringup.launch`
    - **Arguments**:
      - `tf_prefix`: (`value="mir"`) Sets the TF prefix for the MiR robot.
      - `robot_x`, `robot_y`, `robot_yaw`: Initial pose of the MiR robot.
      - `mir_type`: (`value="mir_100"`) Specifies the type of MiR robot (default is MiR 100).

3. **Gazebo World Launch**:
    - **File**: `world.launch`
    - **Arguments**:
      - `world`: Specifies the Gazebo world file to use. Default: **`gazebo_world`**

---

## 2. `multi_robot_nav.launch`
```
roslaunch collaboration_launch multi_robot_nav.launch
```
This launch file sets up the navigation stack for both the DARKO and MiR robots within the multi-robot environment and MoveIt for the DARKO manipulator.
---

### Important Arguments:
- **`map_package`**: (`default="$(find summit_xl_localization)"`) Specifies the ROS package containing the map files.
- **`map_file`**: (`default="willow_garage/willow_garage.yaml"`) Path to the YAML file for the map used by both robots for navigation.
- **`frame_id`**: (`default="map"`) The frame in which the map is published.
- **`launch_mapserver`**: (`default="true"`) Controls whether the map server is launched or not.

### DARKO Robot Navigation Arguments:
- **`id_rbkairos`**: (`default="rbkairos"`) Defines the robot prefix and namespace.
- **`x_init_pose_rbkairos`, `y_init_pose_rbkairos`, `z_init_pose_rbkairos`, `yaw_init_pose_rbkairos`**: Initial pose values for the DARKO robot in the navigation stack.
- **`move_base_rbkairos`**: (`default="true"`) Specifies whether to launch the `move_base` node for navigation.
- **`amcl_rbkairos`**: (`default="true"`) Controls the launch of the AMCL localization node for the DARKO robot.
- **`moveit_movegroup_rbkairos`**: (`default="true"`) Specifies whether to launch MoveIt for motion planning for the robot's arm.

### MiR Robot Navigation Arguments:
- **`tf_prefix`**: (`value="mir"`) Sets the TF prefix for the MiR robot.
- **`robot_x`, `robot_y`**: Specifies the initial position of the MiR robot in the map.

---

### Included Launch Files:

1. **Map Server**:
    - **File**: `map_server.launch`
    - **Arguments**:
      - `map_file`: YAML file for the map. Default: **`willow_garage.yaml`**
      - `maps_path`: Path to the directory containing the map files.
      - `frame_id`: Frame in which the map is published. Default: **`frame_id`**

2. **RB-Kairos Navigation and MoveIt**:
    - **File**: `rbkairos_nav_moveit.launch`
    - **Arguments**:
      - `id_robot`: Prefix & namespace for the DARKO robot.
      - `x_init_pose`, `y_init_pose`, `yaw_init_pose`: Initial pose for DARKO in the map.
      - `launch_move_base`: Controls the launch of the `move_base` navigation node.
      - `launch_amcl`: Launches the AMCL localization node.
      - `launch_move_group`: Launches MoveIt for motion planning.
      - `global_frame`: Reference frame in which robot operates globally within navigation and motion planning. Default: **`frame_id`**

3. **MiR Robot Navigation**:
    - **File**: `mir_nav.launch`
    - **Arguments**:
      - `tf_prefix`: TF prefix for the MiR robot.
      - `robot_x`, `robot_y`: Initial pose for the MiR robot.

---

This setup allows for the simultaneous simulation and navigation of the DARKO and MiR robots in Gazebo, with shared map data, individual navigation capabilities and motion planning functionality for the Panda Arm using MoveIt.
