// scan_data.hpp
// Plain data types passed between the classes of the right wall follower.
// They keep the behaviour code free of ROS types, so it can be read and tested
// without a running ROS graph.

#ifndef TURTLEBOT3_GAZEBO__SCAN_DATA_HPP_
#define TURTLEBOT3_GAZEBO__SCAN_DATA_HPP_

#include <vector>

// The fields of sensor_msgs/LaserScan this program needs. Angles are as the
// LiDAR driver reports them; ScanInterpreter converts them to the robot frame.
struct ScanData
{
  std::vector<float> ranges;      // one range per beam [m]
  double angle_min{0.0};          // angle of ranges[0] [rad]
  double angle_increment{0.0};    // angle between consecutive beams [rad], may be negative
  double range_min{0.0};          // readings below this are not valid [m]
  double range_max{0.0};          // readings at or above this mean "nothing seen" [m]
};

// Everything the wall-following logic looks at for one scan. Distances are
// metres in the robot frame, capped at ScanGeometry::max_range.
struct RangeSummary
{
  double front{0.0};              // nearest obstacle in the frontal cone
  double front_right{0.0};        // range on the diagonal ahead of the right side
  double right{0.0};              // range square to the robot's right
  double wall_distance{0.0};      // perpendicular distance to the wall on the right
  double wall_heading{0.0};       // heading error to that wall [rad];
                                  // positive = nose points away from the wall
  bool heading_valid{false};      // true when both side directions see a wall
  double nearest{0.0};            // closest obstacle in any direction
  double nearest_angle{0.0};      // its robot-frame angle [rad], CCW from the front;
                                  // used to check the LiDAR mounting on the robot
};

#endif  // TURTLEBOT3_GAZEBO__SCAN_DATA_HPP_
