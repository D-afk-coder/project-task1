// turtlebot3_drive.hpp
// Right wall following for the TurtleBot 3. This header declares the ROS node,
// a thin adapter between ROS and the behaviour classes: it turns each
// LaserScan into the program's own types, asks the controller what to do, and
// publishes the answer on cmd_vel. All sensor handling and behaviour logic
// lives in the classes it owns.
//
// Based on turtlebot3_drive.hpp from turtlebot3_simulations,
// Copyright 2019 ROBOTIS CO., LTD., Apache License 2.0. Modified for
// MTRX3760 Project 1.

#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include "turtlebot3_gazebo/scan_interpreter.hpp"
#include "turtlebot3_gazebo/wall_follower_controller.hpp"

class Turtlebot3Drive : public rclcpp::Node
{
public:
  Turtlebot3Drive();

  // Publish a zero velocity. Called by main() on Ctrl-C so the robot stops
  // instead of carrying on with its last command.
  void stop_robot();

private:
  // Build the sensor geometry and controller configuration from parameters.
  ScanGeometry load_geometry();
  WallFollowerConfig load_config();

  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void watchdog_callback();
  void publish_command(const VelocityCommand & command);
  void publish_state(StateId state);
  void report_scan_layout(const sensor_msgs::msg::LaserScan & msg, const ScanData & scan);

  // Declared in initialisation order: the behaviour objects need the parameters.
  ScanInterpreter interpreter_;
  WallFollowerController controller_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // Only one of these two is created, chosen by the use_stamped_cmd_vel parameter.
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_scan_time_;
  double scan_timeout_seconds_{1.0};
  int status_period_ms_{1000};
  bool have_scan_{false};
  bool watchdog_tripped_{false};
};

#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
