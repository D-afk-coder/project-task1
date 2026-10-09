// drive_states.hpp
// The four concrete wall-following behaviours. Right wall following: a wall
// ahead (inner corner) triggers a left turn; a wall that ends (outer corner)
// makes the robot arc right to rejoin it.

#ifndef TURTLEBOT3_GAZEBO__DRIVE_STATES_HPP_
#define TURTLEBOT3_GAZEBO__DRIVE_STATES_HPP_

#include "turtlebot3_gazebo/drive_state.hpp"

class FindWallState : public DriveState
{
public:
  using DriveState::DriveState;
  StateId id() const override { return StateId::FindWall; }
  StateOutput update(const RangeSummary & ranges) override;
};

class FollowWallState : public DriveState
{
public:
  using DriveState::DriveState;
  StateId id() const override { return StateId::FollowWall; }
  StateOutput update(const RangeSummary & ranges) override;
};

class TurnLeftState : public DriveState
{
public:
  using DriveState::DriveState;
  StateId id() const override { return StateId::TurnLeft; }
  StateOutput update(const RangeSummary & ranges) override;
};

class TurnRightState : public DriveState
{
public:
  using DriveState::DriveState;
  StateId id() const override { return StateId::TurnRight; }
  StateOutput update(const RangeSummary & ranges) override;
};

#endif  // TURTLEBOT3_GAZEBO__DRIVE_STATES_HPP_
