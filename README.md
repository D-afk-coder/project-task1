# Member 2: maze simulation and right wall follower

MTRX3760 Project 1, part A1. Everything here drops into the `turtlebot3_gazebo`
package of `turtlebot3_simulations` (jazzy branch), at the same relative paths.

| File | What it is |
| --- | --- |
| `turtlebot3_gazebo/src/turtlebot3_drive.cpp` | The stock drive node, modified into a right wall follower |
| `turtlebot3_gazebo/include/turtlebot3_gazebo/turtlebot3_drive.hpp` | Its header |
| `turtlebot3_gazebo/worlds/turtlebot3_maze.world` | Enclosed 5 x 4 m maze, no floating rooms |
| `turtlebot3_gazebo/launch/turtlebot3_maze.launch.py` | Starts Gazebo, the robot and RViz |
| `turtlebot3_gazebo/rviz/tb3_maze.rviz` | RViz layout: laser scan, camera image, path driven |
| `tools/make_maze_world.py` | Generates the world file; edit the maze here |
| `diagrams/node_diagram.puml`, `.png` | ROS node design diagram (Task 5 standard) |
| `diagrams/uml_class_diagram.puml`, `.png` | Simplified UML class diagram (Lec 2A standard) |

No changes to `CMakeLists.txt` or `package.xml` are needed.

## Build and run

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select turtlebot3_gazebo
source install/setup.bash

# Terminal 1: maze, robot and RViz
export TURTLEBOT3_MODEL=burger_cam     # or waffle_pi; plain "burger" has no camera
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py

# Terminal 2: the wall follower
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_gazebo turtlebot3_drive
```

The robot starts in the south-west cell facing east with the outer wall 0.3 m
to its right, and leaves through the gap in the east wall of the north-east
cell. Expect about a minute and a half. The node prints a line each time it
changes state, with the three distances that caused the change.

## How the wall follower works

Each laser scan is reduced to three distances: front, front-right (a diagonal
ray at 45 degrees) and right. A 20 Hz timer then picks one of four states:

| State | When | What the robot does |
| --- | --- | --- |
| `TURN_LEFT` | Wall ahead closer than 0.32 m | Turns left on the spot until 0.50 m is clear |
| `SEEK_WALL` | No wall to the right or front-right | Arcs right on a 0.30 m radius, round the wall end |
| `FOLLOW_WALL` | Otherwise | Drives at 0.15 m/s, steering to hold a 0.30 m gap |
| `WAIT_FOR_SCAN` | No usable scan for 1 s | Stops |

All the numbers are named constants at the top of `turtlebot3_drive.cpp`.

## Limits worth knowing

- Passages need to be about 0.75 m wide or more with these constants. In
  narrower ones the robot cannot hold a 0.30 m gap and keep 0.32 m clear
  ahead, and it gets stuck turning. For a tighter arena reduce
  `DESIRED_WALL_DIST`, `WALL_LOST_DIST`, `FRONT_BLOCKED_DIST` and
  `FRONT_CLEAR_DIST` together.
- The robot must start with a wall within 0.6 m on its right. With nothing
  there it drives in a small circle looking for one.
- Stopping the node with Ctrl-C does not send a stop command. In Gazebo the
  robot carries on with the last speeds it was given; check what the real
  robot does before relying on Ctrl-C to stop it.

## Differences from simulation already allowed for

- Velocity commands are `TwistStamped`, which is what both the Jazzy
  simulation bridge and the Jazzy `turtlebot3_node` on the robot subscribe to.
  The stock `turtlebot3_drive.cpp` publishes plain `Twist`.
- Laser rays are looked up by angle, not by index, so a laser that does not
  give exactly 360 rays per turn still works.
- Readings of zero, infinity or NaN are ignored.
- The laser subscription uses sensor-data (best effort) quality of service.
