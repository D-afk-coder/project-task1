// drive_states.cpp
// Implementation of the four wall-following behaviours and the factory used by
// the controller to create them without naming the concrete classes.

#include "turtlebot3_gazebo/drive_states.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

const char * to_string(StateId id)
{
  switch (id) {
    case StateId::FindWall:   return "FindWall";
    case StateId::FollowWall: return "FollowWall";
    case StateId::TurnLeft:   return "TurnLeft";
    case StateId::TurnRight:  return "TurnRight";
  }
  return "Unknown";
}

std::unique_ptr<DriveState> make_state(StateId id, const WallFollowerConfig & config)
{
  switch (id) {
    case StateId::FindWall:   return std::make_unique<FindWallState>(config);
    case StateId::FollowWall: return std::make_unique<FollowWallState>(config);
    case StateId::TurnLeft:   return std::make_unique<TurnLeftState>(config);
    case StateId::TurnRight:  return std::make_unique<TurnRightState>(config);
  }
  throw std::invalid_argument("make_state: unknown StateId");
}

// ---------------------------------------------------------------------------
// FindWall: drive forward until a wall shows up on the right or the way is
// blocked ahead.
// ---------------------------------------------------------------------------
StateOutput FindWallState::update(const RangeSummary & ranges)
{
  if (ranges.front < config_.front_stop_distance) {
    return {VelocityCommand::stop(), StateId::TurnLeft};
  }
  // Any wall the follower would not call "lost" is close enough to follow.
  if (ranges.right < config_.wall_lost_distance) {
    return {VelocityCommand{config_.search_speed, 0.0}, StateId::FollowWall};
  }
  return {VelocityCommand{config_.search_speed, 0.0}, StateId::FindWall};
}

// ---------------------------------------------------------------------------
// FollowWall: hold the target distance to the right wall and stay parallel to
// it. Being too far from the wall or having the nose rotated away from it both
// call for a right turn (negative yaw rate).
// ---------------------------------------------------------------------------
StateOutput FollowWallState::update(const RangeSummary & ranges)
{
  if (ranges.front < config_.front_stop_distance) {
    return {VelocityCommand::stop(), StateId::TurnLeft};
  }
  if (ranges.right > config_.wall_lost_distance) {
    return {VelocityCommand{config_.corner_speed, 0.0}, StateId::TurnRight};
  }

  const double distance_error = ranges.wall_distance - config_.target_distance;
  const double heading_error = ranges.heading_valid ? ranges.wall_heading : 0.0;
  const double steering = -(config_.distance_gain * distance_error +
    config_.heading_gain * heading_error);
  const double angular = std::clamp(steering, -config_.max_angular_speed,
      config_.max_angular_speed);

  // Slow down as a wall appears ahead, and while steering hard.
  const double span = config_.front_slow_distance - config_.front_stop_distance;
  const double room = (ranges.front - config_.front_stop_distance) / span;
  const double front_scale = std::clamp(room, 0.3, 1.0);
  const double turn_scale = 1.0 - 0.5 * std::abs(angular) / config_.max_angular_speed;
  const double linear = config_.cruise_speed * front_scale * turn_scale;

  return {VelocityCommand{linear, angular}, StateId::FollowWall};
}

// ---------------------------------------------------------------------------
// TurnLeft: rotate in place until the way ahead is clear beyond the hysteresis
// band and there is a wall on the right to follow.
// ---------------------------------------------------------------------------
StateOutput TurnLeftState::update(const RangeSummary & ranges)
{
  const bool way_is_clear = ranges.front > config_.front_clear_distance;
  // Use the "lost" threshold here too, so this transition cannot flicker with
  // the one in FollowWall and dead ends whose far wall is more than
  // wall_found_distance away still count as walls.
  const bool wall_on_right = ranges.right < config_.wall_lost_distance;
  if (way_is_clear && wall_on_right) {
    return {VelocityCommand::stop(), StateId::FollowWall};
  }
  return {VelocityCommand{0.0, config_.turn_speed}, StateId::TurnLeft};
}

// ---------------------------------------------------------------------------
// TurnRight: arc right at radius = target_distance until the wall reappears,
// which leaves the robot roughly on-target after a quarter circle.
// ---------------------------------------------------------------------------
StateOutput TurnRightState::update(const RangeSummary & ranges)
{
  if (ranges.front < config_.front_stop_distance) {
    return {VelocityCommand::stop(), StateId::TurnLeft};
  }
  if (ranges.heading_valid && ranges.right < config_.wall_found_distance) {
    return {VelocityCommand{config_.corner_speed, 0.0}, StateId::FollowWall};
  }
  const double arc_rate = -config_.corner_speed / config_.target_distance;
  return {VelocityCommand{config_.corner_speed, arc_rate}, StateId::TurnRight};
}
