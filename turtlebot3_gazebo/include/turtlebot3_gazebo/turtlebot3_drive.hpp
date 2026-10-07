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
// turtlebot3_drive.hpp
//
// Declares Turtlebot3Drive, a ROS 2 node that makes a TurtleBot 3 follow the
// wall on its right hand side. The node only deals with ROS; the work is done
// by the two objects it owns:
//
//   Sense.  A CLidar reduces each laser scan to wall distances.
//
//   Decide. A CWallFollower chooses what to do from those distances, and the
//           velocity to do it at.
//
//   Act.    The node publishes that velocity as a velocity command.
//-----------------------------------------------------------------------------

#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "turtlebot3_gazebo/CLidar.h"
#include "turtlebot3_gazebo/CVelocity.h"
#include "turtlebot3_gazebo/CWallFollower.h"

class Turtlebot3Drive : public rclcpp::Node
{
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Turns laser scans into wall distances
  CLidar lidar_;

  // Chooses what to do from the wall distances
  CWallFollower follower_;

  // Sense
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Decide
  void update_callback();

  // Act
  void update_cmd_vel(const CVelocity & velocity);
};
#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
