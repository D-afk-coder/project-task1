# MTRX3760 Project 1: right wall follower

A ROS 2 (Jazzy) node that makes a TurtleBot 3 follow the wall on its right,
in Gazebo and on the real robot, plus the maze world used to test it.
Everything lives in the `turtlebot3_gazebo` package of
`turtlebot3_simulations` (jazzy branch), at the same relative paths.

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

On the robot, start the robot's bringup as usual, then run the same
`ros2 run` command from the laptop.

The node prints a line each time it changes behaviour, with the three
distances that caused the change, for example
`FollowWall -> TurnLeft (front 0.31 m, front-right gap 0.30 m, right 0.32 m)`.

## How the program is structured

One node, `CWallFollowerNode`, is the only class that deals with ROS. The work
is split between the objects it owns:

| Class | Files | Job |
| --- | --- | --- |
| `CWallFollowerNode` | `CWallFollowerNode.h/.cpp` | ROS node: receives scans, runs a 20 Hz update timer, publishes velocity commands |
| `CLidar` | `CLidar.h/.cpp` | Reduces each laser scan to three wall distances; knows when scans have stopped |
| `CWallDistances` | `CWallDistances.h` | The three distances: front, front-right gap, right |
| `CWallFollower` | `CWallFollower.h/.cpp` | Chooses which behaviour is in charge; no ROS types |
| `CDriveBehaviour` | `CDriveBehaviour.h/.cpp` | Abstract base: one way of driving, works out its own velocity |
| `CFollowWall` | `CFollowWall.h/.cpp` | Drive along the wall, steering to hold a 0.30 m gap |
| `CTurnLeft` | `CTurnLeft.h/.cpp` | Turn left on the spot, away from a wall ahead |
| `CSeekWall` | `CSeekWall.h/.cpp` | Arc right round the end of a wall |
| `CStop` | `CStop.h/.cpp` | Stand still when there are no fresh scans |
| `CVelocity` | `CVelocity.h` | A forward speed and turn rate, free of ROS types |

`main` is in `src/turtlebot3_drive.cpp`. Headers are in
`include/turtlebot3_gazebo/`, sources in `src/`.

The behaviours, in order of priority:

| Behaviour | When | What the robot does |
| --- | --- | --- |
| `TurnLeft` | Wall ahead closer than 0.32 m | Turns left on the spot until 0.50 m is clear |
| `SeekWall` | No wall to the right or front-right | Arcs right on a 0.30 m radius, round the wall end |
| `FollowWall` | Otherwise | Drives at 0.15 m/s, steering to hold a 0.30 m gap |
| `Stop` | No usable scan for 1 s | Stops |

## Where to tune

Each tuning value is a named constant in the class that uses it, defined at
the top of that class's `.cpp` file:

| What | Where |
| --- | --- |
| Where the lidar looks (bearings, widths), scan timeout | `src/CLidar.cpp` |
| Decision distances: wall gap, wall lost, front blocked, front clear | `src/CWallFollower.cpp` |
| Following speed, steering gain, turn rate limit | `src/CFollowWall.cpp` |
| Turn-on-the-spot rate | `src/CTurnLeft.cpp` |
| Speed round a wall end | `src/CSeekWall.cpp` |
| Topic names, update rate | `src/CWallFollowerNode.cpp` |

## Limits worth knowing

- Passages need to be about 0.75 m wide or more with these values. In
  narrower ones the robot cannot hold a 0.30 m gap and keep 0.32 m clear
  ahead, and it gets stuck turning. For a tighter arena reduce the four
  distances in `CWallFollower.cpp` together.
- The robot must start with a wall within 0.6 m on its right. With nothing
  there it drives in a small circle looking for one.
- Stopping the node with Ctrl-C does not send a stop command. In Gazebo the
  robot carries on with the last speeds it was given.
- Velocity commands are `TwistStamped`, which is what the Jazzy simulation
  bridge and the Jazzy robot bringup (`enable_stamped_cmd_vel: true`) expect.
  A robot running an older bringup that expects plain `Twist` needs the
  publisher type in `CWallFollowerNode` changed.

## Other files

| File | What it is |
| --- | --- |
| `turtlebot3_gazebo/worlds/turtlebot3_maze.world` | Enclosed 5 x 4 m maze, no floating rooms |
| `turtlebot3_gazebo/launch/turtlebot3_maze.launch.py` | Starts Gazebo, the robot and RViz |
| `turtlebot3_gazebo/rviz/tb3_maze.rviz` | RViz layout: laser scan, camera image, path driven |
| `tools/make_maze_world.py` | Generates the world file; edit the maze here |
| `diagrams/uml_class_diagram.puml`, `.png` | Simplified UML class diagram (Lec 2A standard) |
| `diagrams/node_diagram_sim.puml`, `.png` | ROS node design diagram, simulation (Task 5 standard) |
| `diagrams/node_diagram_robot.puml`, `.png` | ROS node design diagram, physical robot |
