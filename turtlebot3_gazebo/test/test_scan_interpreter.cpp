// test_scan_interpreter.cpp
// Unit tests for ScanInterpreter: wall geometry, invalid readings, and the
// scan layouts of the Gazebo laser, LDS-01, LDS-02 and STL-19P drivers,
// including a LiDAR mounted rotated or spinning clockwise.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "virtual_maze.hpp"
#include "turtlebot3_gazebo/scan_interpreter.hpp"

using wf_test::degToRad;
using wf_test::kPi;
using wf_test::LidarModel;
using wf_test::Pose;
using wf_test::SimulatedLidar;
using wf_test::VirtualMaze;

namespace
{
constexpr double kTolerance = 0.01;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

LidarModel noiseless(LidarModel model)
{
  model.noise_sigma = 0.0;
  model.missing_probability = 0.0;
  return model;
}

ScanData cleanScan(const VirtualMaze & maze, const Pose & pose,
  const LidarModel & model = LidarModel::lds01())
{
  SimulatedLidar lidar(noiseless(model), 1);
  return lidar.scan(maze, pose);
}

// A long straight wall on the robot's right when the robot faces +x.
VirtualMaze rightWall(double distance)
{
  VirtualMaze maze;
  maze.addWall({-10.0, -distance}, {10.0, -distance});
  return maze;
}

// Index of the beam closest to a published (driver-frame) angle in degrees.
std::size_t beamAt(const ScanData & scan, double degrees)
{
  const long n = static_cast<long>(scan.ranges.size());
  const long raw = std::lround((degToRad(degrees) - scan.angle_min) / scan.angle_increment);
  return static_cast<std::size_t>(((raw % n) + n) % n);
}
}  // namespace

// --- Geometry ----------------------------------------------------------------

TEST(ScanInterpreter, ParallelWallGivesDistanceAndZeroHeading)
{
  const ScanInterpreter interpreter;
  const RangeSummary summary = interpreter.summarise(cleanScan(rightWall(0.40), {0, 0, 0}));

  EXPECT_NEAR(summary.right, 0.40, kTolerance);
  EXPECT_NEAR(summary.wall_distance, 0.40, kTolerance);
  EXPECT_NEAR(summary.wall_heading, 0.0, degToRad(1.0));
  EXPECT_TRUE(summary.heading_valid);
  EXPECT_GT(summary.front, 1.4);   // the cone only grazes the side wall far ahead
}

TEST(ScanInterpreter, NoseAwayFromWallGivesPositiveHeading)
{
  const ScanInterpreter interpreter;
  const double yaw = degToRad(10.0);
  const RangeSummary summary = interpreter.summarise(cleanScan(rightWall(0.40), {0, 0, yaw}));

  EXPECT_NEAR(summary.wall_heading, yaw, degToRad(1.5));
  EXPECT_NEAR(summary.wall_distance, 0.40, 0.02);
}

TEST(ScanInterpreter, NoseTowardsWallGivesNegativeHeading)
{
  const ScanInterpreter interpreter;
  const double yaw = degToRad(-15.0);
  const RangeSummary summary = interpreter.summarise(cleanScan(rightWall(0.40), {0, 0, yaw}));

  EXPECT_NEAR(summary.wall_heading, yaw, degToRad(1.5));
  EXPECT_NEAR(summary.wall_distance, 0.40, 0.02);
}

TEST(ScanInterpreter, ObstacleAheadIsReportedInFront)
{
  VirtualMaze maze;
  maze.addWall({0.5, -2.0}, {0.5, 2.0});
  const ScanInterpreter interpreter;
  const RangeSummary summary = interpreter.summarise(cleanScan(maze, {0, 0, 0}));

  EXPECT_NEAR(summary.front, 0.5, kTolerance);
  EXPECT_FALSE(summary.heading_valid);
}

TEST(ScanInterpreter, ObstacleBehindDoesNotCountAsFront)
{
  VirtualMaze maze;
  maze.addWall({-0.3, -2.0}, {-0.3, 2.0});
  const ScanInterpreter interpreter;
  EXPECT_GT(interpreter.summarise(cleanScan(maze, {0, 0, 0})).front, 3.0);
}

TEST(ScanInterpreter, EmptySpaceReportsMaximumRange)
{
  const ScanInterpreter interpreter;
  const RangeSummary summary = interpreter.summarise(cleanScan(VirtualMaze{}, {0, 0, 0}));

  EXPECT_DOUBLE_EQ(summary.front, 3.5);
  EXPECT_DOUBLE_EQ(summary.right, 3.5);
  EXPECT_DOUBLE_EQ(summary.front_right, 3.5);
  EXPECT_FALSE(summary.heading_valid);
}

TEST(ScanInterpreter, WallBeyondSensingRangeIsNotUsedForHeading)
{
  const ScanInterpreter interpreter;
  const RangeSummary summary = interpreter.summarise(cleanScan(rightWall(1.2), {0, 0, 0}));

  EXPECT_NEAR(summary.right, 1.2, kTolerance);
  EXPECT_FALSE(summary.heading_valid);
}

// --- Invalid readings ----------------------------------------------------------

TEST(ScanInterpreter, DropoutOnTheSideBeamIsIgnored)
{
  ScanData scan = cleanScan(rightWall(0.40), {0, 0, 0});
  scan.ranges[beamAt(scan, -90.0)] = 0.0f;
  EXPECT_NEAR(ScanInterpreter{}.summarise(scan).right, 0.40, kTolerance);
}

TEST(ScanInterpreter, SpikeOnTheSideBeamIsRejectedByMedian)
{
  ScanData scan = cleanScan(rightWall(0.40), {0, 0, 0});
  scan.ranges[beamAt(scan, -90.0)] = 0.15f;
  EXPECT_NEAR(ScanInterpreter{}.summarise(scan).right, 0.40, 0.02);
}

TEST(ScanInterpreter, ZeroReadingsAheadAreIgnoredNotTreatedAsWalls)
{
  // The LDS-01 reports 0 for "nothing within 3.5 m". A cone of zeros must not
  // stop the robot (it would spin on the spot in open space).
  ScanData scan = cleanScan(VirtualMaze{}, {0, 0, 0});
  for (int degrees = -20; degrees <= 20; ++degrees) {
    scan.ranges[beamAt(scan, degrees)] = 0.0f;
  }
  EXPECT_DOUBLE_EQ(ScanInterpreter{}.summarise(scan).front, 3.5);
}

TEST(ScanInterpreter, NaNReadingsDoNotHideAnObstacle)
{
  VirtualMaze maze;
  maze.addWall({0.6, -2.0}, {0.6, 2.0});
  ScanData scan = cleanScan(maze, {0, 0, 0});
  for (int degrees = -10; degrees <= 10; degrees += 2) {
    scan.ranges[beamAt(scan, degrees)] = kNaN;
  }
  EXPECT_NEAR(ScanInterpreter{}.summarise(scan).front, 0.6, kTolerance);
}

TEST(ScanInterpreter, NegativeInfinityIsInvalidAndPositiveInfinityIsClear)
{
  ScanData scan = cleanScan(rightWall(0.40), {0, 0, 0}, LidarModel::gazebo());
  scan.ranges[beamAt(scan, 0.0)] = -std::numeric_limits<float>::infinity();
  const RangeSummary summary = ScanInterpreter{}.summarise(scan);
  EXPECT_GT(summary.front, 1.4);
  EXPECT_NEAR(summary.right, 0.40, kTolerance);
}

// --- Driver layouts --------------------------------------------------------------

class DriverLayoutTest : public ::testing::TestWithParam<LidarModel> {};

TEST_P(DriverLayoutTest, SeesWallOnRightAndObstacleAhead)
{
  VirtualMaze maze = rightWall(0.35);
  maze.addWall({0.8, -2.0}, {0.8, 2.0});
  for (unsigned seed = 1; seed <= 5; ++seed) {   // the LDS-02 start angle varies per scan
    SimulatedLidar lidar(noiseless(GetParam()), seed);
    const RangeSummary summary = ScanInterpreter{}.summarise(lidar.scan(maze, {0, 0, 0}));
    EXPECT_NEAR(summary.front, 0.8, 0.03) << GetParam().name;
    EXPECT_NEAR(summary.right, 0.35, 0.02) << GetParam().name;
    EXPECT_NEAR(summary.wall_heading, 0.0, degToRad(3.0)) << GetParam().name;
  }
}

TEST_P(DriverLayoutTest, FrontIsFoundWhenTheScanStartsJustAfterZero)
{
  // Regression: with the LDS-02, angle_min is a little above 0, so "straight
  // ahead" sits past the end of the array. Index-based code reported the
  // front as clear and drove into the wall.
  VirtualMaze maze;
  maze.addWall({0.5, -2.0}, {0.5, 2.0});
  LidarModel model = noiseless(GetParam());
  model.angle_min = degToRad(0.9);
  model.angle_min_jitter = 0.0;
  SimulatedLidar lidar(model, 1);
  EXPECT_NEAR(ScanInterpreter{}.summarise(lidar.scan(maze, {0, 0, 0})).front, 0.5, 0.03);
}

INSTANTIATE_TEST_SUITE_P(
  Drivers, DriverLayoutTest,
  ::testing::Values(LidarModel::gazebo(), LidarModel::lds01(), LidarModel::lds02(),
  LidarModel::stl19p()),
  [](const ::testing::TestParamInfo<LidarModel> & info) {return info.param.name;});

TEST(ScanInterpreter, ScanFromMinusPiToPiWorks)
{
  // Some drivers publish -pi..pi instead of 0..2pi.
  LidarModel model = noiseless(LidarModel::stl19p());
  model.angle_min = -kPi;
  SimulatedLidar lidar(model, 1);
  const RangeSummary summary = ScanInterpreter{}.summarise(lidar.scan(rightWall(0.4), {0, 0, 0}));
  EXPECT_NEAR(summary.right, 0.4, 0.02);
}

TEST(ScanInterpreter, NegativeIncrementWorks)
{
  ScanData scan = cleanScan(rightWall(0.4), {0, 0, 0});
  // Reverse the array and describe it with a negative increment: same scan.
  std::reverse(scan.ranges.begin(), scan.ranges.end());
  scan.angle_min = scan.angle_min + (scan.ranges.size() - 1) * scan.angle_increment;
  scan.angle_increment = -scan.angle_increment;
  EXPECT_NEAR(ScanInterpreter{}.summarise(scan).right, 0.4, kTolerance);
}

TEST(ScanInterpreter, PartialScanCoveringOnlyTheFrontHalf)
{
  ScanData scan;
  scan.angle_min = -kPi / 2.0;
  scan.angle_increment = degToRad(1.0);
  scan.range_min = 0.12;
  scan.range_max = 3.5;
  scan.ranges.assign(181, 2.0f);
  for (int i = 0; i < 181; ++i) {
    const double angle = degToRad(i - 90.0);
    if (angle < -0.01) {
      scan.ranges[static_cast<std::size_t>(i)] =
        static_cast<float>(0.4 / std::cos(angle + kPi / 2.0));
    }
  }
  const ScanInterpreter interpreter;
  ASSERT_TRUE(interpreter.is_usable(scan));
  const RangeSummary summary = interpreter.summarise(scan);
  EXPECT_NEAR(summary.right, 0.4, kTolerance);
  EXPECT_TRUE(summary.heading_valid);
}

// --- Mounting -----------------------------------------------------------------

TEST(ScanInterpreter, RotatedLidarIsCorrectedByYawOffset)
{
  // LiDAR mounted with its zero pointing to the robot's left (+90 deg).
  LidarModel model = noiseless(LidarModel::stl19p());
  model.mount_yaw = degToRad(90.0);
  VirtualMaze maze = rightWall(0.35);
  maze.addWall({0.7, -2.0}, {0.7, 2.0});
  const ScanData scan = SimulatedLidar(model, 1).scan(maze, {0, 0, 0});

  ScanGeometry wrong;   // offset not set: front and right are mixed up
  EXPECT_GT(std::abs(ScanInterpreter{wrong}.summarise(scan).front - 0.7), 0.1);

  ScanGeometry fixed;
  fixed.lidar_yaw_offset = degToRad(90.0);
  const RangeSummary summary = ScanInterpreter{fixed}.summarise(scan);
  EXPECT_NEAR(summary.front, 0.7, 0.03);
  EXPECT_NEAR(summary.right, 0.35, 0.02);
}

TEST(ScanInterpreter, ClockwiseLidarIsCorrectedByFlag)
{
  LidarModel model = noiseless(LidarModel::stl19p());
  model.clockwise = true;
  const ScanData scan = SimulatedLidar(model, 1).scan(rightWall(0.35), {0, 0, 0});

  ScanGeometry fixed;
  fixed.lidar_clockwise = true;
  EXPECT_NEAR(ScanInterpreter{fixed}.summarise(scan).right, 0.35, 0.02);
  // Without the flag the wall looks like it is on the left.
  EXPECT_GT(ScanInterpreter{}.summarise(scan).right, 1.0);
}

TEST(ScanInterpreter, NearestObstacleReportsItsRobotFrameAngle)
{
  // A post 0.3 m away, 40 deg to the left of the robot's front.
  VirtualMaze maze;
  const double a = degToRad(40.0);
  maze.addThickWall({0.3 * std::cos(a), 0.3 * std::sin(a) - 0.02},
    {0.3 * std::cos(a), 0.3 * std::sin(a) + 0.02}, 0.04);
  for (const LidarModel & model : {LidarModel::lds01(), LidarModel::stl19p()}) {
    const RangeSummary summary = ScanInterpreter{}.summarise(cleanScan(maze, {0, 0, 0}, model));
    EXPECT_NEAR(summary.nearest, 0.28, 0.03) << model.name;
    EXPECT_NEAR(summary.nearest_angle, a, degToRad(4.0)) << model.name;
  }
}

TEST(ScanInterpreter, BeamAngleIsReportedInRobotFrame)
{
  ScanData scan = cleanScan(VirtualMaze{}, {0, 0, 0});
  ScanGeometry geometry;
  geometry.lidar_yaw_offset = degToRad(180.0);
  EXPECT_NEAR(std::abs(ScanInterpreter{geometry}.beam_angle(scan, 0)), kPi, 1e-9);
  EXPECT_NEAR(ScanInterpreter{}.beam_angle(scan, 90), kPi / 2.0, 1e-9);
}

// --- Usability ------------------------------------------------------------------

TEST(ScanInterpreter, RejectsUnusableScans)
{
  const ScanInterpreter interpreter;
  ScanData empty;
  EXPECT_FALSE(interpreter.is_usable(empty));

  ScanData no_increment;
  no_increment.ranges.assign(10, 1.0f);
  no_increment.range_max = 3.5;
  EXPECT_FALSE(interpreter.is_usable(no_increment));

  ScanData nan_layout = no_increment;
  nan_layout.angle_increment = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(interpreter.is_usable(nan_layout));
}

TEST(ScanInterpreter, AcceptsZeroRangeMinFromLds02)
{
  ScanData scan = cleanScan(rightWall(0.4), {0, 0, 0}, LidarModel::lds02());
  EXPECT_DOUBLE_EQ(scan.range_min, 0.0);
  EXPECT_TRUE(ScanInterpreter{}.is_usable(scan));
}
