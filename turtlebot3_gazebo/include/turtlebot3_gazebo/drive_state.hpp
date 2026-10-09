// drive_state.hpp
// The State-pattern interface for one wall-following behaviour. Each concrete
// state decides the velocity for the current scan and which state should run
// next. Behaviour changes live in one small class instead of a nest of if
// statements.

#ifndef TURTLEBOT3_GAZEBO__DRIVE_STATE_HPP_
#define TURTLEBOT3_GAZEBO__DRIVE_STATE_HPP_

#include <memory>

#include "turtlebot3_gazebo/scan_data.hpp"
#include "turtlebot3_gazebo/wall_follower_config.hpp"

// Plain velocity command the states return; the node converts it to a ROS
// velocity message.
struct VelocityCommand
{
  double linear{0.0};               // forward speed [m/s]
  double angular{0.0};              // yaw rate, CCW positive [rad/s]

  static VelocityCommand stop() { return VelocityCommand{0.0, 0.0}; }
};

enum class StateId { FindWall, FollowWall, TurnLeft, TurnRight };

const char * to_string(StateId id);

struct StateOutput
{
  VelocityCommand command;          // velocity to apply now
  StateId next;                     // state for the next scan (may be unchanged)
};

class DriveState
{
public:
  explicit DriveState(const WallFollowerConfig & config) : config_(config) {}
  virtual ~DriveState() = default;

  DriveState(const DriveState &) = delete;
  DriveState & operator=(const DriveState &) = delete;

  virtual StateId id() const = 0;
  virtual StateOutput update(const RangeSummary & ranges) = 0;

protected:
  const WallFollowerConfig & config_;   // owned by the controller, which outlives its states
};

// Factory, so the controller never names concrete state classes.
std::unique_ptr<DriveState> make_state(StateId id, const WallFollowerConfig & config);

#endif  // TURTLEBOT3_GAZEBO__DRIVE_STATE_HPP_
