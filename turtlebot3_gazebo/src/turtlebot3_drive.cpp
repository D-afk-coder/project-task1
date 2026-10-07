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
#include <cmath>
#include <functional>
#include <memory>

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
const double DEG2RAD = M_PI / 180.0;
const double FULL_TURN = 2.0 * M_PI;

// Where the robot looks. Bearings are in the laser frame: 0 is straight ahead
// and positive is anticlockwise, so the robot's right hand side is negative.
const double FRONT_BEARING_DEG = 0.0;
const double FRONT_RIGHT_BEARING_DEG = -45.0;
const double RIGHT_BEARING_DEG = -90.0;

// Half the angle each look covers. The front look is wide so that it covers
// the full width of the robot, the front-right look is narrow so that it
// behaves like a single diagonal ray, and the right look is wide enough to
// still find the nearest point of the wall when the robot is not quite
// parallel to it.
const double FRONT_HALF_WIDTH_DEG = 30.0;
const double FRONT_RIGHT_HALF_WIDTH_DEG = 3.0;
const double RIGHT_HALF_WIDTH_DEG = 10.0;

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

// The front-right ray meets a wall on the right further along than the gap
// to it. With the robot parallel to the wall, multiplying the front-right
// reading by this gives the gap back.
const double FRONT_RIGHT_TO_GAP = std::sin(-FRONT_RIGHT_BEARING_DEG * DEG2RAD);

// Timing
const std::chrono::milliseconds UPDATE_PERIOD(50);
const double SCAN_TIMEOUT_SEC = 1.0;      // no scan for this long: stop
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
  front_dist_ = 0.0;
  front_right_dist_ = 0.0;
  right_dist_ = 0.0;

  // Taken from this node's clock so that it can be compared with now() later
  last_scan_time_ = now();
  scan_received_ = false;

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
** Sense: reduce each laser scan to the three distances the decisions need
********************************************************************************/
void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // A scan with no rays or no angular spacing cannot be searched. It is
  // ignored, so the robot stops once the last good scan becomes stale.
  const bool usable = !msg->ranges.empty() && msg->angle_increment > 0.0;

  if (usable) {
    front_dist_ = nearest_in_sector(
      *msg, FRONT_BEARING_DEG, FRONT_HALF_WIDTH_DEG);
    front_right_dist_ = nearest_in_sector(
      *msg, FRONT_RIGHT_BEARING_DEG, FRONT_RIGHT_HALF_WIDTH_DEG);
    right_dist_ = nearest_in_sector(
      *msg, RIGHT_BEARING_DEG, RIGHT_HALF_WIDTH_DEG);

    last_scan_time_ = now();
    scan_received_ = true;
  }
}

//-----------------------------------------------------------------------------
// Returns the nearest valid range [m] within half_width_deg either side of
// bearing_deg. If nothing in the sector gives a valid return, returns the
// laser's maximum range, meaning "nothing seen within range".
//
// Rays are found from the angles the scan itself reports rather than by fixed
// index, because the simulated laser gives exactly 360 rays per turn and a
// real one need not. Readings outside the laser's valid range are skipped:
// Gazebo reports those as infinity and a real laser may report them as zero.
//-----------------------------------------------------------------------------
double Turtlebot3Drive::nearest_in_sector(
  const sensor_msgs::msg::LaserScan & scan,
  double bearing_deg,
  double half_width_deg) const
{
  double nearest = scan.range_max;

  const int ray_count = static_cast<int>(scan.ranges.size());

  // Angle of the sector centre measured from the first ray, within one turn
  double from_first_ray = std::fmod(bearing_deg * DEG2RAD - scan.angle_min, FULL_TURN);
  if (from_first_ray < 0.0) {
    from_first_ray += FULL_TURN;
  }

  const int centre_index = static_cast<int>(std::round(from_first_ray / scan.angle_increment));
  const int half_width = static_cast<int>(
    std::round(half_width_deg * DEG2RAD / scan.angle_increment));

  for (int offset = -half_width; offset <= half_width; ++offset) {
    // The scan is a full circle, so stepping back past the first ray carries
    // on from the last one and the other way round
    const int index = ((centre_index + offset) % ray_count + ray_count) % ray_count;
    const double range = scan.ranges[static_cast<size_t>(index)];

    const bool valid =
      std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;

    if (valid && range < nearest) {
      nearest = range;
    }
  }

  return nearest;
}

//-----------------------------------------------------------------------------
// True while laser scans are still arriving. Driving on old distances would
// mean driving blind, so the caller stops the robot when this is false.
//-----------------------------------------------------------------------------
bool Turtlebot3Drive::scan_is_fresh() const
{
  bool fresh = false;

  if (scan_received_) {
    const double age_sec = (now() - last_scan_time_).seconds();
    fresh = age_sec < SCAN_TIMEOUT_SEC;
  }

  return fresh;
}

/********************************************************************************
** Decide: choose a behaviour from the latest distances
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  double linear = 0.0;
  double angular = 0.0;

  if (scan_is_fresh()) {
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
  DriveState next_state = FOLLOW_WALL;

  // Once turning away from a wall ahead, keep turning until the way is well
  // clear. Starting and stopping the turn at the same distance would leave
  // the robot sitting on the threshold, flicking between two states.
  double blocked_dist = FRONT_BLOCKED_DIST;
  if (state_ == TURN_LEFT) {
    blocked_dist = FRONT_CLEAR_DIST;
  }

  const bool wall_ahead_right = front_right_gap() < WALL_LOST_DIST;
  const bool wall_on_right = right_dist_ < WALL_LOST_DIST;

  // Once rounding the end of a wall, keep rounding it until the front-right
  // ray picks up its far side. The look straight to the right only catches
  // glimpses of the wall end on the way round, too little to steer by.
  const bool still_rounding = (state_ == SEEK_WALL) && !wall_ahead_right;
  const bool wall_gone = !wall_ahead_right && !wall_on_right;

  if (front_dist_ < blocked_dist) {
    next_state = TURN_LEFT;
  } else if (wall_gone || still_rounding) {
    next_state = SEEK_WALL;
  }

  return next_state;
}

//-----------------------------------------------------------------------------
// Returns the gap [m] to the right wall that the front-right reading implies,
// taking the robot to be parallel to the wall.
//
// The value grows both when the robot is further from the wall and when it is
// pointing away from it, and shrinks when it is closer or pointing in. That
// makes it a better thing to steer on than the reading straight to the right,
// which cannot tell which way the robot is pointing.
//-----------------------------------------------------------------------------
double Turtlebot3Drive::front_right_gap() const
{
  return front_right_dist_ * FRONT_RIGHT_TO_GAP;
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
  double gap = right_dist_;
  if (front_right_gap() < WALL_LOST_DIST) {
    gap = front_right_gap();
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
    RCLCPP_INFO(
      get_logger(), "%s -> %s (front %.2f m, front-right %.2f m, right %.2f m)",
      state_name(state_), state_name(new_state),
      front_dist_, front_right_dist_, right_dist_);

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
