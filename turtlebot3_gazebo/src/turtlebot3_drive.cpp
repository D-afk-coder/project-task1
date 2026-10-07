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
#include <cstring>
#include <functional>
#include <memory>

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
const std::chrono::milliseconds UPDATE_PERIOD(50);
const int STALE_WARNING_PERIOD_MS = 2000;

//-----------------------------------------------------------------------------
// Constructor and destructor
//-----------------------------------------------------------------------------
Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
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
** Decide: let the wall follower choose, and report any change of state
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  const bool scan_is_fresh = lidar_.HasFreshScan(now().seconds());
  const char * previous_state = follower_.GetStateName();

  const CVelocity velocity = follower_.Update(lidar_.GetDistances(), scan_is_fresh);

  if (std::strcmp(follower_.GetStateName(), previous_state) != 0) {
    const CWallDistances & distances = lidar_.GetDistances();

    RCLCPP_INFO(
      get_logger(), "%s -> %s (front %.2f m, front-right gap %.2f m, right %.2f m)",
      previous_state, follower_.GetStateName(),
      distances.mFront, distances.mFrontRightGap, distances.mRight);
  }

  if (!scan_is_fresh) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), STALE_WARNING_PERIOD_MS,
      "No usable laser scan: robot stopped");
  }

  update_cmd_vel(velocity);
}

/********************************************************************************
** Act: send the chosen speeds to the robot
********************************************************************************/
void Turtlebot3Drive::update_cmd_vel(const CVelocity & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = now();
  cmd_vel.twist.linear.x = velocity.mLinear;
  cmd_vel.twist.angular.z = velocity.mAngular;

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
