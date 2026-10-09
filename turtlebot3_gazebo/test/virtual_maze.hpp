// virtual_maze.hpp
// Test helper for the right wall follower. A 2D world of walls, a simulated
// LiDAR that can mimic the Gazebo laser, the LDS-01, the LDS-02 and the
// LDROBOT STL-19P (beam layout, invalid readings, mounting angle, spin
// direction), and a differential-drive robot with velocity lag. Passing these
// tests checks the controller logic against each sensor's quirks; it does not
// replace Gazebo or the real robot.

#ifndef TURTLEBOT3_GAZEBO__TEST__VIRTUAL_MAZE_HPP_
#define TURTLEBOT3_GAZEBO__TEST__VIRTUAL_MAZE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "turtlebot3_gazebo/drive_state.hpp"
#include "turtlebot3_gazebo/scan_data.hpp"

namespace wf_test
{
constexpr double kPi = 3.14159265358979323846;
constexpr double degToRad(double degrees) {return degrees * kPi / 180.0;}

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct Pose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// A set of straight, infinitely thin wall segments.
class VirtualMaze
{
public:
  struct Segment
  {
    Point a;
    Point b;
  };

  void addWall(Point a, Point b) {walls_.push_back({a, b});}

  // A solid wall of the given thickness, centred on the line a-b.
  void addThickWall(Point a, Point b, double thickness)
  {
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double nx = -(b.y - a.y) / length * thickness / 2.0;
    const double ny = (b.x - a.x) / length * thickness / 2.0;
    const Point p1{a.x + nx, a.y + ny};
    const Point p2{b.x + nx, b.y + ny};
    const Point p3{b.x - nx, b.y - ny};
    const Point p4{a.x - nx, a.y - ny};
    addWall(p1, p2);
    addWall(p2, p3);
    addWall(p3, p4);
    addWall(p4, p1);
  }

  // Distance along a ray to the first wall, or +inf if there is none.
  double raycast(Point origin, double angle) const
  {
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    double best = std::numeric_limits<double>::infinity();
    for (const Segment & wall : walls_) {
      const double ex = wall.b.x - wall.a.x;
      const double ey = wall.b.y - wall.a.y;
      const double denom = dx * ey - dy * ex;
      if (std::abs(denom) < 1e-12) {
        continue;
      }
      const double qx = wall.a.x - origin.x;
      const double qy = wall.a.y - origin.y;
      const double t = (qx * ey - qy * ex) / denom;
      const double s = (qx * dy - qy * dx) / denom;
      if (t >= 0.0 && s >= 0.0 && s <= 1.0 && t < best) {
        best = t;
      }
    }
    return best;
  }

  // Distance from a point to the nearest wall.
  double clearance(Point p) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (const Segment & wall : walls_) {
      const double ex = wall.b.x - wall.a.x;
      const double ey = wall.b.y - wall.a.y;
      const double length_sq = ex * ex + ey * ey;
      double s = ((p.x - wall.a.x) * ex + (p.y - wall.a.y) * ey) / length_sq;
      s = std::clamp(s, 0.0, 1.0);
      best = std::min(best, std::hypot(p.x - (wall.a.x + s * ex), p.y - (wall.a.y + s * ey)));
    }
    return best;
  }

  const std::vector<Segment> & walls() const {return walls_;}

private:
  std::vector<Segment> walls_;
};

// How a particular LiDAR lays out and reports a scan.
struct LidarModel
{
  std::string name;
  int beams{360};
  int beam_jitter{0};             // beam count varies by up to +/- this per scan
  double angle_min{0.0};          // [rad] as published
  double angle_min_jitter{0.0};   // angle_min varies by up to + this per scan [rad]
  double angle_span{2.0 * kPi};   // total angle covered by the published beams [rad]
  bool span_inclusive{false};     // true: increment = span / (beams - 1)
  double range_min{0.12};
  double range_max{3.5};
  double sensing_limit{3.5};      // beyond this the sensor sees nothing [m]
  float too_close_value{0.0f};    // published when a target is under range_min
  float no_return_value{0.0f};    // published when nothing is seen
  double scan_period{0.2};        // [s]
  double mount_yaw{0.0};          // robot-frame angle of the LiDAR's zero [rad]
  bool clockwise{false};          // published angles increase clockwise
  double noise_sigma{0.01};       // [m]
  double missing_probability{0.01};  // fraction of beams with no reading

  static LidarModel gazebo()
  {
    LidarModel m;
    m.name = "gazebo";
    m.angle_span = 6.28;
    m.span_inclusive = true;
    m.too_close_value = -std::numeric_limits<float>::infinity();
    m.no_return_value = std::numeric_limits<float>::infinity();
    m.missing_probability = 0.0;
    return m;
  }

  static LidarModel lds01()
  {
    LidarModel m;
    m.name = "lds01";
    m.too_close_value = 0.0f;
    m.no_return_value = 0.0f;   // the LDS-01 reports 0 for "nothing seen" too
    return m;
  }

  static LidarModel lds02()
  {
    LidarModel m;
    m.name = "lds02";
    m.beams = 382;
    m.angle_min = degToRad(0.2);
    m.angle_min_jitter = degToRad(0.8);
    m.angle_span = degToRad(359.0);
    m.range_min = 0.0;
    m.range_max = 100.0;
    m.sensing_limit = 8.0;
    m.too_close_value = std::numeric_limits<float>::quiet_NaN();
    m.no_return_value = std::numeric_limits<float>::quiet_NaN();
    m.scan_period = 1.0 / 6.0;
    return m;
  }

  static LidarModel stl19p()
  {
    LidarModel m;
    m.name = "stl19p";
    m.beams = 450;
    m.beam_jitter = 8;
    m.span_inclusive = true;
    m.range_min = 0.02;
    m.range_max = 12.0;
    m.sensing_limit = 12.0;
    m.too_close_value = std::numeric_limits<float>::quiet_NaN();
    m.no_return_value = std::numeric_limits<float>::quiet_NaN();
    m.scan_period = 0.1;
    m.noise_sigma = 0.015;
    m.missing_probability = 0.03;
    return m;
  }
};

class SimulatedLidar
{
public:
  SimulatedLidar(const LidarModel & model, unsigned seed)
  : model_(model), rng_(seed) {}

  ScanData scan(const VirtualMaze & maze, const Pose & pose)
  {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> noise(0.0, std::max(model_.noise_sigma, 1e-12));

    int beams = model_.beams;
    if (model_.beam_jitter > 0) {
      std::uniform_int_distribution<int> jitter(-model_.beam_jitter, model_.beam_jitter);
      beams += jitter(rng_);
    }

    ScanData data;
    data.angle_min = model_.angle_min + model_.angle_min_jitter * uniform(rng_);
    data.angle_increment = model_.angle_span /
      static_cast<double>(model_.span_inclusive ? beams - 1 : beams);
    data.range_min = model_.range_min;
    data.range_max = model_.range_max;
    data.ranges.resize(static_cast<std::size_t>(beams));

    for (int i = 0; i < beams; ++i) {
      const double published = data.angle_min + i * data.angle_increment;
      const double robot_angle = model_.mount_yaw + (model_.clockwise ? -published : published);
      double range = maze.raycast({pose.x, pose.y}, pose.yaw + robot_angle);
      float & out = data.ranges[static_cast<std::size_t>(i)];

      if (std::isinf(range) || range > model_.sensing_limit ||
        uniform(rng_) < model_.missing_probability)
      {
        out = model_.no_return_value;
        continue;
      }
      if (model_.noise_sigma > 0.0) {
        range += noise(rng_);
      }
      if (range < model_.range_min) {
        out = model_.too_close_value;
      } else if (range >= model_.range_max) {
        out = model_.no_return_value;
      } else {
        out = static_cast<float>(range);
      }
    }
    return data;
  }

  double scan_period() const {return model_.scan_period;}

private:
  LidarModel model_;
  std::mt19937 rng_;
};

// Differential-drive robot with first-order velocity lag.
class SimulatedRobot
{
public:
  explicit SimulatedRobot(const Pose & start)
  : pose_(start) {}

  void step(const VelocityCommand & command, double dt)
  {
    const double alpha = 1.0 - std::exp(-dt / kLagSeconds);
    linear_ += (command.linear - linear_) * alpha;
    angular_ += (command.angular - angular_) * alpha;
    pose_.yaw += angular_ * dt;
    pose_.x += linear_ * std::cos(pose_.yaw) * dt;
    pose_.y += linear_ * std::sin(pose_.yaw) * dt;
  }

  const Pose & pose() const {return pose_;}

private:
  static constexpr double kLagSeconds = 0.15;
  Pose pose_;
  double linear_{0.0};
  double angular_{0.0};
};

// A perfect maze (one path between any two cells) with an entrance and an
// exit, built from a seeded depth-first search.
struct GridMaze
{
  VirtualMaze maze;
  Pose start;
  double goal_y{0.0};   // the robot has escaped once its y is above this
};

inline GridMaze makeGridMaze(
  int cols, int rows, double cell, unsigned seed,
  int entrance_col, int exit_col, double stub = 1.0)
{
  struct Cell
  {
    bool visited{false};
    bool wall_east{true};
    bool wall_north{true};
  };
  std::vector<Cell> cells(static_cast<std::size_t>(cols * rows));
  auto at = [&](int c, int r) -> Cell & {
      return cells[static_cast<std::size_t>(r * cols + c)];
    };

  std::mt19937 rng(seed);
  std::vector<std::pair<int, int>> stack{{0, 0}};
  at(0, 0).visited = true;
  while (!stack.empty()) {
    const int c = stack.back().first;
    const int r = stack.back().second;
    std::vector<int> options;   // 0 = east, 1 = north, 2 = west, 3 = south
    if (c + 1 < cols && !at(c + 1, r).visited) {options.push_back(0);}
    if (r + 1 < rows && !at(c, r + 1).visited) {options.push_back(1);}
    if (c > 0 && !at(c - 1, r).visited) {options.push_back(2);}
    if (r > 0 && !at(c, r - 1).visited) {options.push_back(3);}
    if (options.empty()) {
      stack.pop_back();
      continue;
    }
    std::uniform_int_distribution<std::size_t> choose(0, options.size() - 1);
    const int pick = options[choose(rng)];
    int nc = c;
    int nr = r;
    if (pick == 0) {at(c, r).wall_east = false; nc = c + 1;}
    if (pick == 1) {at(c, r).wall_north = false; nr = r + 1;}
    if (pick == 2) {at(c - 1, r).wall_east = false; nc = c - 1;}
    if (pick == 3) {at(c, r - 1).wall_north = false; nr = r - 1;}
    at(nc, nr).visited = true;
    stack.push_back({nc, nr});
  }

  GridMaze result;
  VirtualMaze & maze = result.maze;
  const double width = cols * cell;
  const double height = rows * cell;

  maze.addWall({0.0, 0.0}, {0.0, height});
  maze.addWall({width, 0.0}, {width, height});
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const double x0 = c * cell;
      const double y0 = r * cell;
      if (at(c, r).wall_east && c + 1 < cols) {
        maze.addWall({x0 + cell, y0}, {x0 + cell, y0 + cell});
      }
      if (at(c, r).wall_north && r + 1 < rows) {
        maze.addWall({x0, y0 + cell}, {x0 + cell, y0 + cell});
      }
    }
  }
  for (int c = 0; c < cols; ++c) {
    if (c != entrance_col) {maze.addWall({c * cell, 0.0}, {(c + 1) * cell, 0.0});}
    if (c != exit_col) {maze.addWall({c * cell, height}, {(c + 1) * cell, height});}
  }
  maze.addWall({entrance_col * cell, 0.0}, {entrance_col * cell, -stub});
  maze.addWall({(entrance_col + 1) * cell, 0.0}, {(entrance_col + 1) * cell, -stub});
  maze.addWall({exit_col * cell, height}, {exit_col * cell, height + stub});
  maze.addWall({(exit_col + 1) * cell, height}, {(exit_col + 1) * cell, height + stub});

  result.start = {(entrance_col + 0.5) * cell, -stub + 0.3, kPi / 2.0};
  result.goal_y = height + stub - 0.2;
  return result;
}
}  // namespace wf_test

#endif  // TURTLEBOT3_GAZEBO__TEST__VIRTUAL_MAZE_HPP_
