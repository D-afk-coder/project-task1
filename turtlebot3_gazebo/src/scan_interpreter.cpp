// scan_interpreter.cpp
// Implementation of ScanInterpreter. Each beam's robot-frame angle is computed
// from angle_min, angle_increment, the mounting offset and the spin direction,
// and beams are picked by that angle. A reading is either an obstacle at a
// distance, "clear" (+inf, or at/above range_max) or invalid (NaN, 0, -inf or
// below range_min). Drivers disagree on what invalid means: the LDS-02 and
// STL-19P use NaN for an empty beam, the LDS-01 uses 0 for both "too close"
// and "too far". So invalid readings are simply ignored. The robot never gets
// close enough to a wall for "too close" to matter, because it stops at
// front_stop_distance.

#include "turtlebot3_gazebo/scan_interpreter.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

enum class Reading { Obstacle, Clear, Invalid };

Reading classify(const ScanData & scan, float range)
{
  if (std::isnan(range)) {
    return Reading::Invalid;
  }
  if (std::isinf(range)) {
    return range > 0.0f ? Reading::Clear : Reading::Invalid;
  }
  if (range <= 0.0f || range < scan.range_min) {
    return Reading::Invalid;
  }
  if (range >= scan.range_max) {
    return Reading::Clear;
  }
  return Reading::Obstacle;
}

// Wrap an angle into (-pi, pi].
double normalise(double angle)
{
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle <= 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

// Smallest absolute difference between two angles.
double angle_between(double a, double b)
{
  return std::abs(normalise(a - b));
}
}  // namespace

ScanInterpreter::ScanInterpreter(const ScanGeometry & geometry)
: geometry_(geometry) {}

bool ScanInterpreter::is_usable(const ScanData & scan) const
{
  return !scan.ranges.empty() && std::isfinite(scan.angle_min) &&
         std::isfinite(scan.angle_increment) && scan.angle_increment != 0.0 &&
         std::isfinite(scan.range_max) && scan.range_max > 0.0;
}

double ScanInterpreter::beam_angle(const ScanData & scan, std::size_t index) const
{
  double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
  if (geometry_.lidar_clockwise) {
    angle = -angle;
  }
  return normalise(angle + geometry_.lidar_yaw_offset);
}

RangeSummary ScanInterpreter::summarise(const ScanData & scan) const
{
  const double cap = std::min(geometry_.max_range, scan.range_max);
  const double right_angle = -kPi / 2.0;
  const double diagonal_angle = right_angle + geometry_.diagonal_offset;

  // One pass over the scan: nearest obstacle, front cone minimum, side samples.
  double nearest = cap;
  double nearest_angle = 0.0;
  double front = cap;
  std::vector<double> right_samples;
  std::vector<double> diagonal_samples;

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    const Reading reading = classify(scan, range);
    if (reading == Reading::Invalid) {
      continue;
    }
    const double angle = beam_angle(scan, i);
    const double distance = (reading == Reading::Obstacle) ?
      std::min(static_cast<double>(range), cap) : cap;

    if (reading == Reading::Obstacle && distance < nearest) {
      nearest = distance;
      nearest_angle = angle;
    }
    if (reading == Reading::Obstacle && angle_between(angle, 0.0) <= geometry_.front_half_width) {
      front = std::min(front, distance);
    }
    if (angle_between(angle, right_angle) <= geometry_.side_half_width) {
      right_samples.push_back(distance);
    }
    if (angle_between(angle, diagonal_angle) <= geometry_.side_half_width) {
      diagonal_samples.push_back(distance);
    }
  }

  // Lower median of the side samples rejects single dropouts and spikes, and
  // errs towards "closer".
  auto lower_median = [cap](std::vector<double> & values) {
      if (values.empty()) {
        return cap;
      }
      std::sort(values.begin(), values.end());
      return values[(values.size() - 1) / 2];
    };

  RangeSummary summary;
  summary.nearest = nearest;
  summary.nearest_angle = nearest_angle;
  summary.front = front;
  summary.right = lower_median(right_samples);
  summary.front_right = lower_median(diagonal_samples);

  const bool both_see_wall = summary.right < geometry_.wall_sensing_range &&
    summary.front_right < geometry_.wall_sensing_range;
  if (both_see_wall) {
    // Two rays to a straight wall give its angle and perpendicular distance:
    //   heading  = atan2(a cos(theta) - b, a sin(theta))
    //   distance = b cos(heading)
    // where a is the diagonal range, b the side range and theta their separation.
    const double a = summary.front_right;
    const double b = summary.right;
    const double theta = geometry_.diagonal_offset;
    summary.wall_heading = std::atan2(a * std::cos(theta) - b, a * std::sin(theta));
    summary.wall_distance = b * std::cos(summary.wall_heading);
    summary.heading_valid = true;
  } else {
    summary.wall_heading = 0.0;
    summary.wall_distance = summary.right;
    summary.heading_valid = false;
  }
  return summary;
}
