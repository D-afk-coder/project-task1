// wall_follower_config.cpp
// Validation of WallFollowerConfig values. Catches bad combinations at start-up
// before they turn into bad behaviour (for example, inverted hysteresis bands
// that would make the robot flicker between states at every corner).

#include "turtlebot3_gazebo/wall_follower_config.hpp"

#include <stdexcept>

void WallFollowerConfig::validate() const
{
  if (target_distance <= 0.0) {
    throw std::invalid_argument("target_distance must be positive");
  }
  if (cruise_speed <= 0.0 || corner_speed <= 0.0 || search_speed <= 0.0) {
    throw std::invalid_argument("all forward speeds must be positive");
  }
  if (max_angular_speed <= 0.0 || turn_speed <= 0.0) {
    throw std::invalid_argument("angular speeds must be positive");
  }
  if (!(front_stop_distance < front_clear_distance)) {
    throw std::invalid_argument(
      "front_stop_distance must be below front_clear_distance (hysteresis)");
  }
  if (!(front_stop_distance < front_slow_distance)) {
    throw std::invalid_argument(
      "front_stop_distance must be below front_slow_distance");
  }
  if (!(wall_found_distance < wall_lost_distance)) {
    throw std::invalid_argument(
      "wall_found_distance must be below wall_lost_distance (hysteresis)");
  }
}
