// wall_follower_controller.hpp
// Runs the wall-following state machine. Owns the configuration and the active
// state, with no knowledge of ROS, so the same object can be driven by the
// node, by the maze simulator and by unit tests.

#ifndef TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONTROLLER_HPP_
#define TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONTROLLER_HPP_

#include <memory>

#include "turtlebot3_gazebo/drive_state.hpp"
#include "turtlebot3_gazebo/scan_data.hpp"
#include "turtlebot3_gazebo/wall_follower_config.hpp"

class WallFollowerController
{
public:
  // Throws std::invalid_argument if the configuration is inconsistent.
  explicit WallFollowerController(const WallFollowerConfig & config);

  // States hold a reference into this object, so copying/moving would dangle.
  WallFollowerController(const WallFollowerController &) = delete;
  WallFollowerController & operator=(const WallFollowerController &) = delete;
  WallFollowerController(WallFollowerController &&) = delete;
  WallFollowerController & operator=(WallFollowerController &&) = delete;

  VelocityCommand update(const RangeSummary & ranges);
  StateId state() const;
  const WallFollowerConfig & config() const { return config_; }

private:
  void switch_to(StateId next);

  WallFollowerConfig config_;         // declared first: states refer to it
  std::unique_ptr<DriveState> state_;
};

#endif  // TURTLEBOT3_GAZEBO__WALL_FOLLOWER_CONTROLLER_HPP_
