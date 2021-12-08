# Description

The `move_server.py` node starts a ROS service server that moves the arm along a given trajectory.

# Move and Open Gripper

Given a waypoints file, the `move_and_open_gripper.py` node will move the robot along the given trajectory, then open the gripper when the movement is complete.

## Usage

### With launchfile

```
roslaunch move_robot move_and_open_gripper.launch waypoints:=waypoints/waypoints.npy
```

# Move and Collect Data

Given a waypoints file, the `move_and_collect.py` node handles the collection of data when executing the trajectory. More specifically, it starts collecting data at the start of the movement, stops collection of data at the end of the movement, and saves logs upon completion.

Currently, raw camera output and end-effector pose can be collected.

## Usage

### With launchfile

Example: Collect end-effector pose and camera data:

```
roslaunch move_robot move_and_collect.launch waypoints:=waypoints/waypoints.npy out_dir:=output flags:='--log-pose --log-camera'
```

Example: Don't collect data:

```
roslaunch move_robot move_and_collect.launch waypoints:=waypoints/waypoints.npy
```

### Manual Usage

Start the `move` service:

```
rosrun move_robot move_server.py
```

Call the `move` service:

```
rosservice call move "waypoints: 'waypoints/waypoints.npy' control_freq: 20"
```

Call the `move` service through `move_and_collect.py`:

```
rosrun move_robot move_and_collect.py -w waypoints/waypoints.npy
```

Use the `--help` flag to see additional arguments.