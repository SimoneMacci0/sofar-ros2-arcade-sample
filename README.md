# sofar-ros2-arcade-sample
Sample ROS2 Python package demonstrating a simple Arcade simulation environment controlled via ROS2

## Dependencies

The project targets ROS2 distributions. It has been successfully tested with Galactic and Humble distros (desktop installation).

The only external depencency needed is Arcade library (see [Instructions for Linux install](https://api.arcade.academy/en/latest/install/linux.html))

## Execution

Clone the repository in your workspace and compile as usual.

Run the simulation node with the command:

```ros2 launch ros2_arcade_sample arcade_sim.launch.py```

Send waypoints to action server through CLI as follows:

```ros2 action send_goal /waypoints ros2_arcade_sample_interface2/action/WaypointsAction "{waypoints: [{x: 650, y: 500, z: 0}, {x: 200, y: 200, z: 0}]}"```
