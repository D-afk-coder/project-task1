// test_maze_sim.cpp
// End-to-end tests for the right wall follower: the real ScanInterpreter and
// WallFollowerController drive a simulated robot through the Gazebo maze
// (worlds/mtrx_maze.world), random enclosed mazes, an open track and awkward
// starting poses, with each LiDAR's quirks and noise, without touching a wall.
// Set WALL_FOLLOWER_TRAJECTORY_DIR to also write a CSV of each run.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <functional>
#include <string>

#include "maze_runner.hpp"
#include "sim_maze_layout.hpp"

using wf_test::degToRad;
using wf_test::GridMaze;
using wf_test::kPi;
using wf_test::LidarModel;
using wf_test::makeGridMaze;
using wf_test::Pose;
using wf_test::RunResult;
using wf_test::RunSettings;
using wf_test::runInMaze;
using wf_test::VirtualMaze;

namespace
{
std::string trajectoryPath(const std::string & name)
{
  const char * dir = std::getenv("WALL_FOLLOWER_TRAJECTORY_DIR");
  return dir ? std::string(dir) + "/" + name + ".csv" : std::string();
}

RunSettings settingsFor(const std::string & name, const LidarModel & lidar, unsigned seed)
{
  RunSettings settings;
  settings.lidar = lidar;
  settings.seed = seed;
  settings.trajectory_csv = trajectoryPath(name);
  return settings;
}

VirtualMaze makeGazeboMaze()
{
  VirtualMaze maze;
  for (const auto & w : wf_test::kSimMazeWalls) {
    // Gazebo boxes extend half a thickness past each end of the centre line.
    const double length = std::hypot(w.x2 - w.x1, w.y2 - w.y1);
    const double ex = (w.x2 - w.x1) / length * wf_test::kWallThickness / 2.0;
    const double ey = (w.y2 - w.y1) / length * wf_test::kWallThickness / 2.0;
    maze.addThickWall({w.x1 - ex, w.y1 - ey}, {w.x2 + ex, w.y2 + ey}, wf_test::kWallThickness);
  }
  return maze;
}

// An L-shaped block in open space, like the "open track" in the brief.
VirtualMaze makeOpenTrack()
{
  VirtualMaze maze;
  maze.addWall({0.0, 0.0}, {2.0, 0.0});
  maze.addWall({2.0, 0.0}, {2.0, 1.6});
  maze.addWall({2.0, 1.6}, {1.2, 1.6});
  maze.addWall({1.2, 1.6}, {1.2, 0.8});
  maze.addWall({1.2, 0.8}, {0.0, 0.8});
  maze.addWall({0.0, 0.8}, {0.0, 0.0});
  return maze;
}

// True once the robot has left `start` and come back to it `laps` times.
class LapCounter
{
public:
  LapCounter(const Pose & start, int laps)
  : start_(start), laps_(laps) {}

  bool operator()(const Pose & pose)
  {
    const double distance = std::hypot(pose.x - start_.x, pose.y - start_.y);
    if (distance > 1.5) {
      away_ = true;
    }
    if (away_ && distance < 0.4) {
      ++completed_;
      away_ = false;
    }
    return completed_ >= laps_;
  }

  int completed() const {return completed_;}

private:
  Pose start_;
  int laps_;
  int completed_{0};
  bool away_{false};
};

// A LiDAR mounted turned 90 deg to the left and spinning clockwise, with the
// interpreter told about both.
struct OddMount
{
  LidarModel lidar;
  ScanGeometry geometry;
};

OddMount oddMount()
{
  OddMount m{LidarModel::stl19p(), ScanGeometry{}};
  m.lidar.name = "stl19p_rotated_clockwise";
  m.lidar.mount_yaw = degToRad(90.0);
  m.lidar.clockwise = true;
  m.geometry.lidar_yaw_offset = degToRad(90.0);
  m.geometry.lidar_clockwise = true;
  return m;
}
}  // namespace

// ---------------------------------------------------------------------------
// The Gazebo maze, with every LiDAR
// ---------------------------------------------------------------------------
class GazeboMazeTest : public ::testing::TestWithParam<LidarModel> {};

TEST_P(GazeboMazeTest, EscapesTheMazeWithoutTouchingAWall)
{
  const VirtualMaze maze = makeGazeboMaze();
  const Pose start{wf_test::kSpawnX, wf_test::kSpawnY, 0.0};
  const RunResult result = runInMaze(
    maze, start, [](const Pose & p) {return p.x > wf_test::kExitX;},
    WallFollowerConfig{}, settingsFor("gazebo_maze_" + GetParam().name, GetParam(), 3));

  EXPECT_TRUE(result.finished) << "still inside after " << result.time << " s";
  EXPECT_EQ(result.collisions, 0) << "closest approach " << result.min_clearance << " m";
}

INSTANTIATE_TEST_SUITE_P(
  Lidars, GazeboMazeTest,
  ::testing::Values(LidarModel::gazebo(), LidarModel::lds01(), LidarModel::lds02(),
  LidarModel::stl19p()),
  [](const ::testing::TestParamInfo<LidarModel> & info) {return info.param.name;});

TEST(GazeboMaze, EscapesWithRotatedClockwiseLidarOnceConfigured)
{
  const OddMount mount = oddMount();
  RunSettings settings = settingsFor("gazebo_maze_odd_mount", mount.lidar, 5);
  settings.geometry = mount.geometry;
  const RunResult result = runInMaze(
    makeGazeboMaze(), {wf_test::kSpawnX, wf_test::kSpawnY, 0.0},
    [](const Pose & p) {return p.x > wf_test::kExitX;}, WallFollowerConfig{}, settings);

  EXPECT_TRUE(result.finished);
  EXPECT_EQ(result.collisions, 0);
}

TEST(GazeboMaze, EscapesWithSlowerCautiousTuning)
{
  WallFollowerConfig cautious;
  cautious.cruise_speed = 0.10;
  cautious.corner_speed = 0.08;
  cautious.search_speed = 0.08;
  const RunResult result = runInMaze(
    makeGazeboMaze(), {wf_test::kSpawnX, wf_test::kSpawnY, 0.0},
    [](const Pose & p) {return p.x > wf_test::kExitX;}, cautious,
    settingsFor("gazebo_maze_cautious", LidarModel::stl19p(), 9));

  EXPECT_TRUE(result.finished);
  EXPECT_EQ(result.collisions, 0);
}

// ---------------------------------------------------------------------------
// Random enclosed mazes
// ---------------------------------------------------------------------------
struct MazeCase
{
  unsigned seed;
  double cell;
  LidarModel lidar;
};

class RandomMazeTest : public ::testing::TestWithParam<MazeCase> {};

TEST_P(RandomMazeTest, SolvesMazeWithoutTouchingAWall)
{
  const MazeCase c = GetParam();
  const GridMaze grid = makeGridMaze(8, 6, c.cell, c.seed, 0, 7);
  const double goal_y = grid.goal_y;
  const RunResult result = runInMaze(
    grid.maze, grid.start, [goal_y](const Pose & p) {return p.y > goal_y;},
    WallFollowerConfig{},
    settingsFor("random_" + c.lidar.name + "_" + std::to_string(c.seed), c.lidar, c.seed));

  EXPECT_TRUE(result.finished) << "did not escape within " << result.time << " s";
  EXPECT_EQ(result.collisions, 0) << "closest approach " << result.min_clearance << " m";
}

INSTANTIATE_TEST_SUITE_P(
  Seeds, RandomMazeTest,
  ::testing::Values(
    MazeCase{1, 0.8, LidarModel::stl19p()}, MazeCase{2, 0.8, LidarModel::stl19p()},
    MazeCase{3, 0.8, LidarModel::stl19p()}, MazeCase{4, 0.8, LidarModel::stl19p()},
    MazeCase{5, 0.7, LidarModel::stl19p()}, MazeCase{6, 0.7, LidarModel::stl19p()},
    MazeCase{7, 1.0, LidarModel::stl19p()}, MazeCase{8, 1.0, LidarModel::stl19p()},
    MazeCase{11, 0.8, LidarModel::lds01()}, MazeCase{12, 0.8, LidarModel::lds02()},
    MazeCase{13, 0.8, LidarModel::gazebo()}, MazeCase{14, 0.6, LidarModel::stl19p()}));

// ---------------------------------------------------------------------------
// Open track
// ---------------------------------------------------------------------------
TEST(OpenTrack, CirclesAnObstacleWithoutTouchingIt)
{
  const Pose start{1.0, -0.30, kPi};   // heading west, block on the right
  LapCounter laps(start, 2);
  const RunResult result = runInMaze(
    makeOpenTrack(), start, std::ref(laps), WallFollowerConfig{},
    settingsFor("open_track", LidarModel::stl19p(), 7));

  EXPECT_TRUE(result.finished) << "completed " << laps.completed() << " laps";
  EXPECT_EQ(result.collisions, 0) << "closest approach " << result.min_clearance << " m";
}

// ---------------------------------------------------------------------------
// Imperfect starting poses
// ---------------------------------------------------------------------------
struct StartPose
{
  double wall_distance;   // [m] from the block's lower edge
  double yaw_error_deg;   // rotation away from "parallel to the wall"
};

class StartPoseTest : public ::testing::TestWithParam<StartPose> {};

TEST_P(StartPoseTest, FindsTheWallAndFollowsItAroundTheBlock)
{
  const StartPose p = GetParam();
  const Pose start{1.0, -p.wall_distance, kPi + degToRad(p.yaw_error_deg)};
  LapCounter laps(Pose{1.0, -0.30, kPi}, 1);
  const RunResult result = runInMaze(
    makeOpenTrack(), start, std::ref(laps), WallFollowerConfig{},
    settingsFor("start_pose", LidarModel::stl19p(), 3));

  EXPECT_TRUE(result.finished);
  EXPECT_EQ(result.collisions, 0) << "closest approach " << result.min_clearance << " m";
}

INSTANTIATE_TEST_SUITE_P(
  Poses, StartPoseTest,
  ::testing::Values(
    StartPose{0.15, 0.0}, StartPose{0.30, 0.0}, StartPose{0.50, 0.0},
    StartPose{0.30, 30.0}, StartPose{0.30, -30.0}, StartPose{0.50, 30.0},
    StartPose{0.20, -30.0}));
