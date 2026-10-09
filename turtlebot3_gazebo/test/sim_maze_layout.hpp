// sim_maze_layout.hpp
// Wall layout of worlds/mtrx_maze.world, generated from the same list, so the
// end-to-end tests drive the wall follower through exactly the maze used in
// Gazebo. Each wall is a centre line from (x1, y1) to (x2, y2) in metres; the
// Gazebo boxes are kWallThickness thick and extend kWallThickness / 2 past
// each end.

#ifndef TURTLEBOT3_GAZEBO__TEST__SIM_MAZE_LAYOUT_HPP_
#define TURTLEBOT3_GAZEBO__TEST__SIM_MAZE_LAYOUT_HPP_

#include <array>

namespace wf_test
{
struct WallLine
{
  double x1, y1, x2, y2;
};

constexpr double kWallThickness = 0.06;
constexpr double kSpawnX = -0.6;    // matches launch/mtrx_maze.launch.py
constexpr double kSpawnY = 0.5;
constexpr double kExitX = 5.8;      // the robot has left the maze past this x

constexpr std::array<WallLine, 15> kSimMazeWalls{{
  {-1.00, 0.00, 5.00, 0.00},
  {-1.00, 1.00, 0.00, 1.00},
  {4.00, 1.00, 5.00, 1.00},
  {1.00, 2.00, 3.00, 2.00},
  {3.00, 3.00, 4.00, 3.00},
  {5.00, 3.00, 6.00, 3.00},
  {0.00, 4.00, 6.00, 4.00},
  {-1.00, 0.00, -1.00, 1.00},
  {0.00, 1.00, 0.00, 4.00},
  {1.00, 0.00, 1.00, 3.00},
  {2.00, 0.00, 2.00, 1.00},
  {2.00, 3.00, 2.00, 4.00},
  {3.00, 1.00, 3.00, 3.00},
  {4.00, 2.00, 4.00, 3.00},
  {5.00, 0.00, 5.00, 3.00}
}};
}  // namespace wf_test

#endif  // TURTLEBOT3_GAZEBO__TEST__SIM_MAZE_LAYOUT_HPP_
