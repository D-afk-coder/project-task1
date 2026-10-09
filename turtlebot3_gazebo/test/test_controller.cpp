// test_controller.cpp
// Unit tests for the right wall follower's state machine: each DriveState's
// decisions and transitions, the controller that runs them, and the checks on
// WallFollowerConfig.

#include <gtest/gtest.h>

#include <stdexcept>
#include <type_traits>

#include "turtlebot3_gazebo/wall_follower_controller.hpp"

namespace {

/// Build a summary where the side beams are consistent with a wall.
RangeSummary summary(double front, double right, double heading = 0.0, bool heading_valid = true) {
  RangeSummary s;
  s.front = front;
  s.right = right;
  s.front_right = right * 1.4;
  s.wall_distance = right;
  s.wall_heading = heading;
  s.heading_valid = heading_valid;
  return s;
}

/// Controller already in FollowWall (one scan beside a wall gets it there).
class FollowingController {
 public:
  FollowingController() : controller_(WallFollowerConfig{}) {
    controller_.update(summary(3.5, 0.30));
  }
  WallFollowerController& get() { return controller_; }

 private:
  WallFollowerController controller_;
};

}  // namespace

TEST(Controller, StartsLookingForAWall) {
  WallFollowerController controller{WallFollowerConfig{}};
  EXPECT_EQ(controller.state(), StateId::FindWall);
}

TEST(Controller, DrivesStraightWhileNoWallIsNearby) {
  const WallFollowerConfig config;
  WallFollowerController controller(config);
  const VelocityCommand command = controller.update(summary(3.5, 3.5, 0.0, false));

  EXPECT_EQ(controller.state(), StateId::FindWall);
  EXPECT_DOUBLE_EQ(command.linear, config.search_speed);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(Controller, StartsFollowingOnceAWallIsOnTheRight) {
  WallFollowerController controller{WallFollowerConfig{}};
  controller.update(summary(3.5, 0.30));
  EXPECT_EQ(controller.state(), StateId::FollowWall);
}

TEST(Controller, TurnsLeftIfBlockedWhileSearching) {
  WallFollowerController controller{WallFollowerConfig{}};
  const VelocityCommand command = controller.update(summary(0.20, 3.5, 0.0, false));

  EXPECT_EQ(controller.state(), StateId::TurnLeft);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_GT(command.angular, 0.0);
}

TEST(Following, HoldsStraightAtTargetDistanceWhenParallel) {
  FollowingController fixture;
  const VelocityCommand command = fixture.get().update(summary(3.5, 0.30, 0.0));

  EXPECT_NEAR(command.angular, 0.0, 1e-9);
  EXPECT_NEAR(command.linear, fixture.get().config().cruise_speed, 1e-9);
}

TEST(Following, SteersRightWhenTooFarFromWall) {
  FollowingController fixture;
  EXPECT_LT(fixture.get().update(summary(3.5, 0.45, 0.0)).angular, 0.0);
}

TEST(Following, SteersLeftWhenTooCloseToWall) {
  FollowingController fixture;
  EXPECT_GT(fixture.get().update(summary(3.5, 0.20, 0.0)).angular, 0.0);
}

TEST(Following, SteersRightWhenNosePointsAwayFromWall) {
  FollowingController fixture;
  EXPECT_LT(fixture.get().update(summary(3.5, 0.30, 0.3)).angular, 0.0);
}

TEST(Following, SteersLeftWhenNosePointsTowardsWall) {
  FollowingController fixture;
  EXPECT_GT(fixture.get().update(summary(3.5, 0.30, -0.3)).angular, 0.0);
}

TEST(Following, IgnoresHeadingWhenItIsNotValid) {
  FollowingController fixture;
  const VelocityCommand command = fixture.get().update(summary(3.5, 0.30, 0.9, false));
  EXPECT_NEAR(command.angular, 0.0, 1e-9);
}

TEST(Following, SteeringIsLimited) {
  FollowingController fixture;
  const double limit = fixture.get().config().max_angular_speed;
  EXPECT_GE(fixture.get().update(summary(3.5, 0.55, 1.5)).angular, -limit - 1e-9);
  EXPECT_LE(fixture.get().update(summary(3.5, 0.12, -1.5)).angular, limit + 1e-9);
}

TEST(Following, SlowsDownApproachingAWall) {
  FollowingController fixture;
  const double open_speed = fixture.get().update(summary(3.5, 0.30)).linear;
  const double near_speed = fixture.get().update(summary(0.45, 0.30)).linear;
  EXPECT_LT(near_speed, open_speed);
  EXPECT_GT(near_speed, 0.0);
}

TEST(Following, StopsAndTurnsLeftAtAnInnerCorner) {
  FollowingController fixture;
  const VelocityCommand command = fixture.get().update(summary(0.30, 0.30));

  EXPECT_EQ(fixture.get().state(), StateId::TurnLeft);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_GT(command.angular, 0.0);
}

TEST(Following, ArcsRightWhenTheWallEnds) {
  FollowingController fixture;
  const WallFollowerConfig& config = fixture.get().config();
  const VelocityCommand command = fixture.get().update(summary(3.5, 1.0, 0.0, false));

  EXPECT_EQ(fixture.get().state(), StateId::TurnRight);
  EXPECT_DOUBLE_EQ(command.linear, config.corner_speed);
  EXPECT_NEAR(command.angular, -config.corner_speed / config.target_distance, 1e-9);
}

TEST(TurningLeft, KeepsTurningWhileBlocked) {
  FollowingController fixture;
  fixture.get().update(summary(0.30, 0.30));
  fixture.get().update(summary(0.30, 0.30));
  EXPECT_EQ(fixture.get().state(), StateId::TurnLeft);
}

TEST(TurningLeft, NeedsTheFrontToBeClearBeyondTheHysteresisBand) {
  FollowingController fixture;
  fixture.get().update(summary(0.30, 0.30));  // enter TurnLeft
  const double between = 0.5 * (fixture.get().config().front_stop_distance +
                                fixture.get().config().front_clear_distance);
  fixture.get().update(summary(between, 0.30));
  EXPECT_EQ(fixture.get().state(), StateId::TurnLeft);
}

TEST(TurningLeft, ResumesFollowingOnceClearWithAWallOnTheRight) {
  FollowingController fixture;
  fixture.get().update(summary(0.30, 0.30));
  const VelocityCommand command = fixture.get().update(summary(1.5, 0.30));

  EXPECT_EQ(fixture.get().state(), StateId::FollowWall);
  EXPECT_GT(command.linear, 0.0);
}

TEST(TurningLeft, KeepsTurningInADeadEndWhoseFarWallIsFurtherThanWallFound) {
  // Regression: a 0.8 m wide dead end leaves the robot 0.5 m from the wall that
  // ends up on its right after turning. That must still count as a wall.
  FollowingController fixture;
  fixture.get().update(summary(0.30, 0.30));
  fixture.get().update(summary(1.5, 0.50));
  EXPECT_EQ(fixture.get().state(), StateId::FollowWall);
}

TEST(TurningLeft, DoesNotResumeWithoutAWallOnTheRight) {
  FollowingController fixture;
  fixture.get().update(summary(0.30, 0.30));
  fixture.get().update(summary(1.5, 2.0, 0.0, false));
  EXPECT_EQ(fixture.get().state(), StateId::TurnLeft);
}

TEST(TurningRight, KeepsArcingUntilTheWallIsFoundAgain) {
  FollowingController fixture;
  fixture.get().update(summary(3.5, 1.0, 0.0, false));
  fixture.get().update(summary(3.5, 1.0, 0.0, false));
  EXPECT_EQ(fixture.get().state(), StateId::TurnRight);
}

TEST(TurningRight, ResumesFollowingWhenTheWallReappears) {
  FollowingController fixture;
  fixture.get().update(summary(3.5, 1.0, 0.0, false));
  fixture.get().update(summary(3.5, 0.35, 0.2, true));
  EXPECT_EQ(fixture.get().state(), StateId::FollowWall);
}

TEST(TurningRight, TurnsLeftIfSomethingBlocksTheArc) {
  FollowingController fixture;
  fixture.get().update(summary(3.5, 1.0, 0.0, false));
  fixture.get().update(summary(0.25, 1.0, 0.0, false));
  EXPECT_EQ(fixture.get().state(), StateId::TurnLeft);
}

TEST(Config, DefaultsAreConsistent) {
  EXPECT_NO_THROW(WallFollowerConfig{}.validate());
}

TEST(Config, RejectsBadHysteresisBands) {
  WallFollowerConfig config;
  config.front_clear_distance = config.front_stop_distance;
  EXPECT_THROW(WallFollowerController{config}, std::invalid_argument);

  config = WallFollowerConfig{};
  config.wall_found_distance = config.wall_lost_distance;
  EXPECT_THROW(WallFollowerController{config}, std::invalid_argument);
}

TEST(Config, RejectsNonPositiveValues) {
  WallFollowerConfig config;
  config.target_distance = 0.0;
  EXPECT_THROW(config.validate(), std::invalid_argument);

  config = WallFollowerConfig{};
  config.cruise_speed = -0.1;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(Controller, CannotBeCopiedOrMoved) {
  static_assert(!std::is_copy_constructible<WallFollowerController>::value, "must not copy");
  static_assert(!std::is_move_constructible<WallFollowerController>::value, "must not move");
  SUCCEED();
}

TEST(StateNames, AreReadable) {
  EXPECT_STREQ(to_string(StateId::FindWall), "FindWall");
  EXPECT_STREQ(to_string(StateId::FollowWall), "FollowWall");
  EXPECT_STREQ(to_string(StateId::TurnLeft), "TurnLeft");
  EXPECT_STREQ(to_string(StateId::TurnRight), "TurnRight");
}
