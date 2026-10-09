// wall_follower_config.hpp
// Every tunable number used by the wall-following behaviour. One place to look
// when the robot needs re-tuning, and one struct to validate at start-up so a
// bad combination fails loudly instead of making the robot chatter at corners.

#ifndef TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONFIG_HPP_
#define TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONFIG_HPP_

struct WallFollowerConfig
{
  // Following the wall
  double target_distance{0.30};        // wanted distance to the right wall [m]
  double cruise_speed{0.15};           // forward speed on a clear run [m/s]
  double max_angular_speed{1.0};       // steering is clamped to this [rad/s]
  double distance_gain{2.0};           // steering per metre of distance error
  double heading_gain{1.5};            // steering per radian of heading error

  // Wall ahead (inner corner)
  double front_slow_distance{0.70};    // start slowing down below this [m]
  double front_stop_distance{0.35};    // stop and turn left below this [m]
  double front_clear_distance{0.55};   // the turn ends once the front is clear beyond this [m]
  double turn_speed{0.8};              // in-place left turn rate [rad/s]

  // Wall ends (outer corner)
  double wall_lost_distance{0.60};     // right wall counts as lost beyond this [m]
  double wall_found_distance{0.45};    // right wall counts as found within this [m]
  double corner_speed{0.12};           // forward speed while arcing round a corner [m/s]

  // Looking for a first wall
  double search_speed{0.12};           // forward speed while no wall is nearby [m/s]

  // Throws std::invalid_argument if the values are inconsistent (for example
  // if a hysteresis band would be inverted).
  void validate() const;
};

#endif  // TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONFIG_HPP_
