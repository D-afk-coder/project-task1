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
// wall on its right hand side. The original ROBOTIS node drove around avoiding
// obstacles; this version keeps its structure (one node, a laser subscriber,
// a velocity publisher and an update timer) and replaces the behaviour.
//
// The node works in three steps, each in its own group of member functions:
//
//   Sense.  Each laser scan is reduced to three distances: the nearest thing
//           ahead, to the front-right on the diagonal, and directly to the
//           right.
//
//   Decide. The update timer picks a state from those distances. A wall ahead
//           means turn left, no wall on the right means the wall has ended so
//           arc right round its end, and otherwise follow the wall.
//
//   Act.    The state is turned into a forward and a turning speed, which are
//           published as a velocity command.
//
// The node assumes the robot starts with a wall on its right.
//-----------------------------------------------------------------------------

#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Turtlebot3Drive : public rclcpp::Node
{
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // What the robot is doing about the wall on its right
  enum DriveState
  {
    WAIT_FOR_SCAN,  // no usable laser data yet, or it has stopped: stay still
    FOLLOW_WALL,    // wall on the right: hold a set distance from it
    TURN_LEFT,      // wall ahead: turn on the spot until the way is clear
    SEEK_WALL       // wall on the right has ended: arc right round its end
  };

  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Nearest obstacle in each direction of interest, from the latest scan [m]
  double front_dist_;
  double front_right_dist_;
  double right_dist_;

  // When the latest usable scan arrived, so a silent lidar stops the robot
  rclcpp::Time last_scan_time_;
  bool scan_received_;

  // Current behaviour
  DriveState state_;

  // Sense
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  double nearest_in_sector(
    const sensor_msgs::msg::LaserScan & scan,
    double bearing_deg,
    double half_width_deg) const;
  bool scan_is_fresh() const;

  // Decide
  void update_callback();
  DriveState choose_state() const;
  double front_right_gap() const;
  double follow_wall_turn_rate() const;
  void change_state(DriveState new_state);
  const char * state_name(DriveState state) const;

  // Act
  void update_cmd_vel(double linear, double angular);
};
#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
