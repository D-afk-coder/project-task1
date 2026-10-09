// wall_follower_controller.cpp
// Runs the wall-following state machine. One scan goes in, one velocity comes
// out. If a state wants to switch, the new state runs on the same scan so the
// robot does not idle for one scan period (200 ms at 5 Hz) at every transition.

#include "turtlebot3_gazebo/wall_follower_controller.hpp"

WallFollowerController::WallFollowerController(const WallFollowerConfig & config)
: config_(config), state_()
{
  config_.validate();
  switch_to(StateId::FindWall);
}

VelocityCommand WallFollowerController::update(const RangeSummary & ranges)
{
  StateOutput output = state_->update(ranges);

  if (output.next != state_->id()) {
    switch_to(output.next);
    output = state_->update(ranges);
    if (output.next != state_->id()) {
      switch_to(output.next);   // a second switch in one scan is deferred
    }
  }
  return output.command;
}

StateId WallFollowerController::state() const { return state_->id(); }

void WallFollowerController::switch_to(StateId next)
{
  state_ = make_state(next, config_);
}
