// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified for MTRX3760 Project 1, "Hello, Turtlebots!".

//-----------------------------------------------------------------------------
// turtlebot3_drive.cpp
//
// Implements Turtlebot3Drive, the right wall follower. See the header for how
// the node is organised.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <chrono>
#include <functional>
#include <memory>

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

// Distances the decisions are based on [m], measured from the laser
const double DESIRED_WALL_DIST = 0.30;   // gap to hold from the right wall
const double WALL_LOST_DIST = 0.60;      // right wall further than this: ended
const double FRONT_BLOCKED_DIST = 0.32;  // wall ahead nearer than this: turn
const double FRONT_CLEAR_DIST = 0.50;    // keep turning until this is clear

// Speeds [m/s] and turn rates [rad/s]
const double LINEAR_VELOCITY = 0.15;         // following a wall
const double CORNER_LINEAR_VELOCITY = 0.10;  // rounding the end of a wall
const double TURN_ANGULAR_VELOCITY = 0.6;    // turning on the spot
const double MAX_FOLLOW_ANGULAR_VELOCITY = 0.8;

// Turn rate per metre of error in the gap to the wall [rad/s per m]
const double STEERING_GAIN = 3.0;

// Timing
const std::chrono::milliseconds UPDATE_PERIOD(50);
const int STALE_WARNING_PERIOD_MS = 2000;

//-----------------------------------------------------------------------------
// Constructor and destructor
//-----------------------------------------------------------------------------
Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  state_ = WAIT_FOR_SCAN;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers. ROS 2 Jazzy TurtleBot3 packages, in simulation and
  // on the robot, expect a stamped velocity command.
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot3Drive::scan_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = create_wall_timer(
    UPDATE_PERIOD, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(get_logger(), "Turtlebot3 right wall follower has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(get_logger(), "Turtlebot3 right wall follower has been terminated");
}

/********************************************************************************
** Sense: hand each laser scan to the lidar
********************************************************************************/
void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  lidar_.ReadScan(*msg, now().seconds());
}

/********************************************************************************
** Decide: choose a behaviour from the latest distances
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  double linear = 0.0;
  double angular = 0.0;

  if (lidar_.HasFreshScan(now().seconds())) {
    change_state(choose_state());
  } else {
    change_state(WAIT_FOR_SCAN);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), STALE_WARNING_PERIOD_MS,
      "No usable laser scan: robot stopped");
  }

  switch (state_) {
    case FOLLOW_WALL:
      linear = LINEAR_VELOCITY;
      angular = follow_wall_turn_rate();
      break;

    case TURN_LEFT:
      linear = 0.0;
      angular = TURN_ANGULAR_VELOCITY;
      break;

    case SEEK_WALL:
      // Turning at speed / radius drives a circle of that radius. A radius
      // equal to the desired gap carries the robot round the end of the wall
      // it has just passed, at the same gap it was following it at.
      linear = CORNER_LINEAR_VELOCITY;
      angular = -CORNER_LINEAR_VELOCITY / DESIRED_WALL_DIST;
      break;

    case WAIT_FOR_SCAN:
    default:
      linear = 0.0;
      angular = 0.0;
      break;
  }

  update_cmd_vel(linear, angular);
}

//-----------------------------------------------------------------------------
// Picks the behaviour for the current distances, most urgent first.
//-----------------------------------------------------------------------------
Turtlebot3Drive::DriveState Turtlebot3Drive::choose_state() const
{
  const CWallDistances & distances = lidar_.GetDistances();
  DriveState next_state = FOLLOW_WALL;

  // Once turning away from a wall ahead, keep turning until the way is well
  // clear. Starting and stopping the turn at the same distance would leave
  // the robot sitting on the threshold, flicking between two states.
  double blocked_dist = FRONT_BLOCKED_DIST;
  if (state_ == TURN_LEFT) {
    blocked_dist = FRONT_CLEAR_DIST;
  }

  const bool wall_ahead_right = distances.mFrontRightGap < WALL_LOST_DIST;
  const bool wall_on_right = distances.mRight < WALL_LOST_DIST;

  // Once rounding the end of a wall, keep rounding it until the front-right
  // ray picks up its far side. The look straight to the right only catches
  // glimpses of the wall end on the way round, too little to steer by.
  const bool still_rounding = (state_ == SEEK_WALL) && !wall_ahead_right;
  const bool wall_gone = !wall_ahead_right && !wall_on_right;

  if (distances.mFront < blocked_dist) {
    next_state = TURN_LEFT;
  } else if (wall_gone || still_rounding) {
    next_state = SEEK_WALL;
  }

  return next_state;
}

//-----------------------------------------------------------------------------
// Returns the turn rate [rad/s] that holds the desired gap to the right wall.
//
// Steers on the front-right gap whenever the front-right ray can see the wall,
// so that the heading is corrected along with the gap and the robot settles
// instead of weaving. When that ray looks past the end of the wall, the
// reading straight to the right is used instead.
//-----------------------------------------------------------------------------
double Turtlebot3Drive::follow_wall_turn_rate() const
{
  const CWallDistances & distances = lidar_.GetDistances();

  double gap = distances.mRight;
  if (distances.mFrontRightGap < WALL_LOST_DIST) {
    gap = distances.mFrontRightGap;
  }

  // Too far from the wall gives a positive error, and the wall is on the
  // right, so the correction is a turn to the right: a negative turn rate
  double turn_rate = -STEERING_GAIN * (gap - DESIRED_WALL_DIST);

  if (turn_rate > MAX_FOLLOW_ANGULAR_VELOCITY) {
    turn_rate = MAX_FOLLOW_ANGULAR_VELOCITY;
  } else if (turn_rate < -MAX_FOLLOW_ANGULAR_VELOCITY) {
    turn_rate = -MAX_FOLLOW_ANGULAR_VELOCITY;
  }

  return turn_rate;
}

//-----------------------------------------------------------------------------
// Moves to a new state, reporting the change and the distances that caused it.
//-----------------------------------------------------------------------------
void Turtlebot3Drive::change_state(DriveState new_state)
{
  if (new_state != state_) {
    const CWallDistances & distances = lidar_.GetDistances();

    RCLCPP_INFO(
      get_logger(), "%s -> %s (front %.2f m, front-right gap %.2f m, right %.2f m)",
      state_name(state_), state_name(new_state),
      distances.mFront, distances.mFrontRightGap, distances.mRight);

    state_ = new_state;
  }
}

const char * Turtlebot3Drive::state_name(DriveState state) const
{
  const char * name = "UNKNOWN";

  switch (state) {
    case WAIT_FOR_SCAN:
      name = "WAIT_FOR_SCAN";
      break;

    case FOLLOW_WALL:
      name = "FOLLOW_WALL";
      break;

    case TURN_LEFT:
      name = "TURN_LEFT";
      break;

    case SEEK_WALL:
      name = "SEEK_WALL";
      break;

    default:
      break;
  }

  return name;
}

/********************************************************************************
** Act: send the chosen speeds to the robot
********************************************************************************/
void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = now();
  cmd_vel.twist.linear.x = linear;
  cmd_vel.twist.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
