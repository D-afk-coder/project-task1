# MTRX3760 Project 1: right wall follower

This is the stock ROBOTIS `turtlebot3_gazebo` package (Jazzy branch) with
`turtlebot3_drive` replaced by a right wall follower. The rest of the upstream
package (worlds, models, launch files, bridge configs) is unchanged, so this
folder can stand in for the apt-installed package when built in a workspace.

## Files added or changed

| File | What it is |
|---|---|
| `src/turtlebot3_drive.cpp`, `include/turtlebot3_gazebo/turtlebot3_drive.hpp` | The ROS node (modified) |
| `include/.../scan_interpreter.hpp`, `src/scan_interpreter.cpp` | LaserScan to front / right / wall angle |
| `include/.../drive_state.hpp`, `drive_states.hpp`, `src/drive_states.cpp` | The four behaviours (State pattern) |
| `include/.../wall_follower_controller.hpp`, `src/wall_follower_controller.cpp` | Runs the state machine |
| `include/.../wall_follower_config.hpp`, `src/wall_follower_config.cpp` | Tuning values and their checks |
| `include/.../scan_data.hpp` | Plain data types shared by the above |
| `params/wall_follower.yaml` | Every parameter, with defaults |
| `launch/wall_follower.launch.py` | Runs the wall follower |
| `launch/mtrx_maze.launch.py`, `worlds/mtrx_maze.world` | Gazebo maze for the simulation hurdle |
| `test/` | Unit tests and simulated-maze tests |
| `docs/` | Node design and class diagrams (Graphviz source and PNG) |
| `CMakeLists.txt`, `package.xml` | Upstream files plus the above |

## Quick reference

```bash
# Simulation
export TURTLEBOT3_MODEL=burger_cam
ros2 launch turtlebot3_gazebo mtrx_maze.launch.py
ros2 launch turtlebot3_gazebo wall_follower.launch.py use_sim_time:=true

# Real robot (bring-up already running on the robot)
ros2 launch turtlebot3_gazebo wall_follower.launch.py

# Emergency stop (Jazzy uses TwistStamped on cmd_vel)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"

# Tests
colcon test --packages-select turtlebot3_gazebo --event-handlers console_direct+
```

Ctrl-C on the wall follower also stops the robot. The node stops the robot by
itself if scans stop arriving for `scan_timeout` seconds.

## Topics

| Topic | Type | |
|---|---|---|
| `scan` | `sensor_msgs/msg/LaserScan` | subscribed |
| `cmd_vel` | `geometry_msgs/msg/TwistStamped` (or `Twist` if `use_stamped_cmd_vel: false`) | published |
| `wall_follower/state` | `std_msgs/msg/String` | published on each behaviour change |

## Known limits

* Right wall following cannot solve a maze with a floating room (out of scope
  for Project 1).
* Corridors narrower than about 0.6 m are not reliable.
* There is no stuck detection: a robot wedged on an obstacle keeps trying.
