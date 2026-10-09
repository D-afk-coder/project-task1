// scan_interpreter.hpp
// Reduces a raw laser scan to the few distances the right wall follower uses.
// Every assumption about the LiDAR lives here: how it is mounted, which way it
// spins, how it reports invalid readings, and which directions count as
// "ahead" and "to the right". Beams are chosen by angle, never by index, so the
// same code works with the Gazebo laser, the LDS-01, the LDS-02 and the
// LDROBOT STL-19P, whatever their beam count or starting angle.

#ifndef TURTLEBOT3_GAZEBO__SCAN_INTERPRETER_HPP_
#define TURTLEBOT3_GAZEBO__SCAN_INTERPRETER_HPP_

#include <cstddef>

#include "turtlebot3_gazebo/scan_data.hpp"

// Tunable sensor geometry. Angles are radians, distances metres.
struct ScanGeometry
{
  double front_half_width{0.2618};    // half-angle of the frontal cone (15 deg)
  double diagonal_offset{0.7854};     // diagonal direction is this far ahead of right (45 deg)
  double side_half_width{0.0524};     // side directions use the median within +/- this (3 deg)
  double wall_sensing_range{0.8};     // a wall farther than this is ignored for heading
  double max_range{3.5};              // every distance is capped at this
  double lidar_yaw_offset{0.0};       // robot-frame angle of the LiDAR's 0 direction (CCW +)
  bool lidar_clockwise{false};        // true if the scan's angles increase clockwise
};

class ScanInterpreter
{
public:
  explicit ScanInterpreter(const ScanGeometry & geometry = ScanGeometry{});

  // True if the scan has beams and a usable angular layout.
  bool is_usable(const ScanData & scan) const;

  // Summarise a usable scan. Behaviour is unspecified if is_usable() is false.
  RangeSummary summarise(const ScanData & scan) const;

  // Robot-frame angle of beam `index`, in (-pi, pi]. Public so the node can
  // report the scan layout once at start-up.
  double beam_angle(const ScanData & scan, std::size_t index) const;

private:
  ScanGeometry geometry_;
};

#endif  // TURTLEBOT3_GAZEBO__SCAN_INTERPRETER_HPP_
