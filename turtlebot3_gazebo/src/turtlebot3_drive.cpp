// turtlebot3_drive.cpp
// Right wall following for the TurtleBot 3. The node subscribes to scan, hands
// each scan to a ScanInterpreter and then to a WallFollowerController, and
// publishes the resulting velocity on cmd_vel (TwistStamped by default, as
// ROS 2 Jazzy expects). A watchdog stops the robot if scans stop arriving, and
// Ctrl-C stops the robot before the node exits. Every tunable number is a ROS
// parameter; see params/wall_follower.yaml.
//
// Based on turtlebot3_drive.cpp from turtlebot3_simulations,
// Copyright 2019 ROBOTIS CO., LTD., Apache License 2.0. Modified for
// MTRX3760 Project 1.

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <functional>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
}  // namespace

Turtlebot3Drive::Turtlebot3Drive()
: rclcpp::Node("turtlebot3_drive_node"),
  interpreter_(load_geometry()),
  controller_(load_config()),
  last_scan_time_(0, 0, RCL_ROS_TIME)
{
  scan_timeout_seconds_ = this->declare_parameter<double>("scan_timeout", 1.0);
  const double status_period = this->declare_parameter<double>("status_period", 1.0);
  status_period_ms_ = static_cast<int>(status_period * 1000.0);
  const bool stamped = this->declare_parameter<bool>("use_stamped_cmd_vel", true);

  // Best effort matches every LiDAR driver and the Gazebo bridge.
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot3Drive::scan_callback, this, std::placeholders::_1));

  const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (stamped) {
    cmd_vel_stamped_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", cmd_qos);
  } else {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", cmd_qos);
  }
  state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "wall_follower/state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  watchdog_timer_ = this->create_wall_timer(
    100ms, std::bind(&Turtlebot3Drive::watchdog_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Right wall follower ready: target wall distance %.2f m, cruise %.2f m/s, "
    "cmd_vel type %s. Waiting for scans...",
    controller_.config().target_distance, controller_.config().cruise_speed,
    stamped ? "TwistStamped" : "Twist");
}

ScanGeometry Turtlebot3Drive::load_geometry()
{
  ScanGeometry g;
  g.front_half_width = kDegToRad * this->declare_parameter<double>(
    "front_half_width_deg", g.front_half_width * kRadToDeg);
  g.diagonal_offset = kDegToRad * this->declare_parameter<double>(
    "diagonal_offset_deg", g.diagonal_offset * kRadToDeg);
  g.side_half_width = kDegToRad * this->declare_parameter<double>(
    "side_half_width_deg", g.side_half_width * kRadToDeg);
  g.wall_sensing_range = this->declare_parameter<double>(
    "wall_sensing_range", g.wall_sensing_range);
  g.max_range = this->declare_parameter<double>("max_range", g.max_range);
  g.lidar_yaw_offset = kDegToRad * this->declare_parameter<double>(
    "lidar_yaw_offset_deg", 0.0);
  g.lidar_clockwise = this->declare_parameter<bool>("lidar_clockwise", false);
  return g;
}

WallFollowerConfig Turtlebot3Drive::load_config()
{
  WallFollowerConfig c;
  c.target_distance = this->declare_parameter<double>("target_distance", c.target_distance);
  c.cruise_speed = this->declare_parameter<double>("cruise_speed", c.cruise_speed);
  c.max_angular_speed = this->declare_parameter<double>(
    "max_angular_speed", c.max_angular_speed);
  c.distance_gain = this->declare_parameter<double>("distance_gain", c.distance_gain);
  c.heading_gain = this->declare_parameter<double>("heading_gain", c.heading_gain);
  c.front_slow_distance = this->declare_parameter<double>(
    "front_slow_distance", c.front_slow_distance);
  c.front_stop_distance = this->declare_parameter<double>(
    "front_stop_distance", c.front_stop_distance);
  c.front_clear_distance = this->declare_parameter<double>(
    "front_clear_distance", c.front_clear_distance);
  c.turn_speed = this->declare_parameter<double>("turn_speed", c.turn_speed);
  c.wall_lost_distance = this->declare_parameter<double>(
    "wall_lost_distance", c.wall_lost_distance);
  c.wall_found_distance = this->declare_parameter<double>(
    "wall_found_distance", c.wall_found_distance);
  c.corner_speed = this->declare_parameter<double>("corner_speed", c.corner_speed);
  c.search_speed = this->declare_parameter<double>("search_speed", c.search_speed);
  return c;
}

void Turtlebot3Drive::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  ScanData scan;
  scan.ranges = msg->ranges;
  scan.angle_min = msg->angle_min;
  scan.angle_increment = msg->angle_increment;
  scan.range_min = msg->range_min;
  scan.range_max = msg->range_max;

  if (!interpreter_.is_usable(scan)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Ignoring a LaserScan with no usable beam layout");
    return;
  }
  if (!have_scan_) {
    report_scan_layout(*msg, scan);
  }

  last_scan_time_ = this->now();
  have_scan_ = true;
  watchdog_tripped_ = false;

  const StateId before = controller_.state();
  const RangeSummary ranges = interpreter_.summarise(scan);
  const VelocityCommand command = controller_.update(ranges);
  const StateId after = controller_.state();

  if (after != before) {
    RCLCPP_INFO(this->get_logger(), "State: %s -> %s", to_string(before), to_string(after));
    publish_state(after);
  }
  if (status_period_ms_ > 0) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), status_period_ms_,
      "[%s] front %.2f  right %.2f  wall %.2f m @ %+.0f deg  ->  v %.2f  w %+.2f  "
      "| nearest %.2f m @ %+.0f deg",
      to_string(after), ranges.front, ranges.right, ranges.wall_distance,
      ranges.wall_heading * kRadToDeg, command.linear, command.angular,
      ranges.nearest, ranges.nearest_angle * kRadToDeg);
  }
  publish_command(command);
}

void Turtlebot3Drive::report_scan_layout(
  const sensor_msgs::msg::LaserScan & msg, const ScanData & scan)
{
  long no_return = 0;
  long dropout = 0;
  long clear = 0;
  for (const float r : scan.ranges) {
    if (std::isnan(r)) {
      ++no_return;
    } else if (std::isinf(r) ? r > 0.0f : r >= scan.range_max) {
      ++clear;
    } else if (std::isinf(r) || r <= 0.0f || r < scan.range_min) {
      ++dropout;
    }
  }
  const double span = std::abs(scan.angle_increment) * static_cast<double>(scan.ranges.size());
  RCLCPP_INFO(
    this->get_logger(),
    "First scan (frame '%s'): %zu beams from %.1f deg, step %.2f deg (%.0f deg covered), "
    "range %.2f-%.1f m. Beam 0 points %.1f deg from the robot's front. "
    "%ld no-return (NaN), %ld dropout, %ld clear.",
    msg.header.frame_id.c_str(), scan.ranges.size(), scan.angle_min * kRadToDeg,
    scan.angle_increment * kRadToDeg, span * kRadToDeg, scan.range_min, scan.range_max,
    interpreter_.beam_angle(scan, 0) * kRadToDeg, no_return, dropout, clear);
  if (span < 1.9 * 3.14159265358979323846) {
    RCLCPP_WARN(
      this->get_logger(),
      "The scan covers less than a full circle. Make sure it covers the front and right.");
  }
}

void Turtlebot3Drive::watchdog_callback()
{
  if (!have_scan_ || watchdog_tripped_) {
    return;
  }
  const double age = (this->now() - last_scan_time_).seconds();
  if (age > scan_timeout_seconds_) {
    watchdog_tripped_ = true;
    publish_command(VelocityCommand::stop());
    RCLCPP_WARN(this->get_logger(), "No scan for %.1f s: robot stopped", age);
  }
}

void Turtlebot3Drive::stop_robot()
{
  publish_command(VelocityCommand::stop());
}

void Turtlebot3Drive::publish_command(const VelocityCommand & command)
{
  if (cmd_vel_stamped_pub_) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.twist.linear.x = command.linear;
    msg.twist.angular.z = command.angular;
    cmd_vel_stamped_pub_->publish(msg);
  } else {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = command.linear;
    msg.angular.z = command.angular;
    cmd_vel_pub_->publish(msg);
  }
}

void Turtlebot3Drive::publish_state(StateId state)
{
  std_msgs::msg::String msg;
  msg.data = to_string(state);
  state_pub_->publish(msg);
}

/*******************************************************************************
** Main
** ROS's own Ctrl-C handler is replaced by one that only sets a flag, so the
** node can still publish a stop command before ROS shuts down.
*******************************************************************************/
namespace
{
std::atomic<bool> g_stop_requested{false};

void on_signal(int)
{
  g_stop_requested = true;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::shared_ptr<Turtlebot3Drive> node;
  try {
    node = std::make_shared<Turtlebot3Drive>();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("turtlebot3_drive"), "Cannot start: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !g_stop_requested) {
    executor.spin_once(100ms);
  }

  // Send the stop a few times so it is not lost if a subscriber is slow.
  for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
    node->stop_robot();
    std::this_thread::sleep_for(50ms);
  }
  RCLCPP_INFO(node->get_logger(), "Shutting down: robot told to stop");
  rclcpp::shutdown();
  return 0;
}
