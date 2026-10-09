// maze_runner.hpp
// Test helper for the right wall follower. Closes the loop between the
// simulated LiDAR and the real ScanInterpreter and WallFollowerController,
// one scan at a time with one scan of latency, so the end-to-end tests drive
// the same code the robot runs.

#ifndef TURTLEBOT3_GAZEBO__TEST__MAZE_RUNNER_HPP_
#define TURTLEBOT3_GAZEBO__TEST__MAZE_RUNNER_HPP_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <string>

#include "virtual_maze.hpp"
#include "turtlebot3_gazebo/drive_state.hpp"
#include "turtlebot3_gazebo/scan_interpreter.hpp"
#include "turtlebot3_gazebo/wall_follower_controller.hpp"

namespace wf_test
{
struct RunSettings
{
  LidarModel lidar{LidarModel::lds01()};
  ScanGeometry geometry{};           // how the interpreter is configured
  double max_time{900.0};            // give up after this much simulated time [s]
  double robot_radius{0.105};        // TurtleBot 3 Burger footprint [m]
  unsigned seed{1};
  std::string trajectory_csv;        // optional path to log t,x,y,yaw,state
};

struct RunResult
{
  bool finished{false};
  double time{0.0};
  int collisions{0};                 // scans on which the robot overlapped a wall
  double min_clearance{1e9};         // smallest centre-to-wall distance [m]
  double path_length{0.0};
  std::map<StateId, int> state_counts;
};

// Run the controller until `finished(pose)` is true or time runs out.
inline RunResult runInMaze(
  const VirtualMaze & maze, const Pose & start,
  const std::function<bool(const Pose &)> & finished,
  const WallFollowerConfig & config, const RunSettings & settings)
{
  SimulatedLidar lidar(settings.lidar, settings.seed);
  SimulatedRobot robot(start);
  ScanInterpreter interpreter(settings.geometry);
  WallFollowerController controller(config);

  std::ofstream csv;
  if (!settings.trajectory_csv.empty()) {
    csv.open(settings.trajectory_csv);
    csv << "t,x,y,yaw,state\n";
    std::ofstream walls(settings.trajectory_csv + ".walls");
    for (const auto & wall : maze.walls()) {
      walls << wall.a.x << ',' << wall.a.y << ',' << wall.b.x << ',' << wall.b.y << '\n';
    }
  }

  RunResult result;
  VelocityCommand command = VelocityCommand::stop();
  const double period = lidar.scan_period();
  const int substeps = 10;
  const double dt = period / substeps;

  for (double t = 0.0; t < settings.max_time; t += period) {
    // The command worked out from this scan is applied during the next period.
    const ScanData scan = lidar.scan(maze, robot.pose());
    const VelocityCommand next_command = controller.update(interpreter.summarise(scan));
    ++result.state_counts[controller.state()];

    for (int i = 0; i < substeps; ++i) {
      const Pose before = robot.pose();
      robot.step(command, dt);
      result.path_length += std::hypot(robot.pose().x - before.x, robot.pose().y - before.y);
    }
    command = next_command;

    const double clearance = maze.clearance({robot.pose().x, robot.pose().y});
    result.min_clearance = std::min(result.min_clearance, clearance);
    if (clearance < settings.robot_radius) {
      ++result.collisions;
    }
    if (csv.is_open()) {
      csv << t << ',' << robot.pose().x << ',' << robot.pose().y << ',' << robot.pose().yaw <<
        ',' << to_string(controller.state()) << '\n';
    }
    result.time = t;
    if (finished(robot.pose())) {
      result.finished = true;
      break;
    }
  }
  return result;
}
}  // namespace wf_test

#endif  // TURTLEBOT3_GAZEBO__TEST__MAZE_RUNNER_HPP_
