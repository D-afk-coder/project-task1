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

//-----------------------------------------------------------------------------
// CWallFollowerNode.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// Connects CLidar and CWallFollower to ROS. Scans are read as they arrive; a
// timer then runs the wall follower at a steady rate, whether or not a new
// scan has come in, so that the robot stops if the laser goes quiet.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CWallFollowerNode.h"

#include <functional>

//---Settings------------------------------------------------------------------
const std::string CWallFollowerNode::kNodeName = "wall_follower";
const std::string CWallFollowerNode::kScanTopic = "scan";
const std::string CWallFollowerNode::kVelocityTopic = "cmd_vel";
const int CWallFollowerNode::kVelocityQueueDepth = 10;
const std::chrono::milliseconds CWallFollowerNode::kUpdatePeriod( 50 );
const int CWallFollowerNode::kStaleWarningPeriodMs = 2000;


//-----------------------------------------------------------------------------
// The velocity command is stamped because the ROS 2 Jazzy TurtleBot 3
// packages, in simulation and on the robot, expect TwistStamped. The scan is
// subscribed with sensor-data quality of service to match the laser drivers.
//-----------------------------------------------------------------------------
CWallFollowerNode::CWallFollowerNode()
    :
        Node( kNodeName )
{
    mpVelocityPublisher = create_publisher<geometry_msgs::msg::TwistStamped>(
        kVelocityTopic,
        rclcpp::QoS( rclcpp::KeepLast( kVelocityQueueDepth ) ) );

    mpScanSubscription = create_subscription<sensor_msgs::msg::LaserScan>(
        kScanTopic,
        rclcpp::SensorDataQoS(),
        std::bind( &CWallFollowerNode::OnScan, this, std::placeholders::_1 ) );

    mpUpdateTimer = create_wall_timer(
        kUpdatePeriod,
        std::bind( &CWallFollowerNode::OnUpdateTimer, this ) );

    RCLCPP_INFO( get_logger(), "Right wall follower started" );
}


//-----------------------------------------------------------------------------
CWallFollowerNode::~CWallFollowerNode()
{
    RCLCPP_INFO( get_logger(), "Right wall follower stopped" );
}


//-----------------------------------------------------------------------------
void CWallFollowerNode::OnScan(
    const sensor_msgs::msg::LaserScan::SharedPtr apScan )
{
    if( apScan != nullptr )
    {
        const double TimeSec =
            now().seconds();

        mLidar.ReadScan(
            *apScan,
            TimeSec );
    }
}


//-----------------------------------------------------------------------------
// Runs the wall-following controller using the latest laser measurements.
// stale measurements cause CWallFollower to select the Stop behaviour.
void CWallFollowerNode::OnUpdateTimer()
{
    const double NowSec =
        now().seconds();

    const bool ScanIsFresh =
        mLidar.HasFreshScan(
            NowSec );

    const std::string PreviousBehaviour =
        mFollower.GetBehaviourName();

    const CVelocity Velocity =
        mFollower.Update(
            mLidar.GetDistances(),
            ScanIsFresh );

    if(
        mFollower.GetBehaviourName() !=
        PreviousBehaviour )
    {
        ReportBehaviourChange(
            PreviousBehaviour );
    }

    if( !ScanIsFresh )
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            kStaleWarningPeriodMs,
            "No usable laser scan: robot stopped" );
    }

    PublishVelocity(
        Velocity );
}


//-----------------------------------------------------------------------------
void CWallFollowerNode::PublishVelocity( const CVelocity& arVelocity )
{
    geometry_msgs::msg::TwistStamped Command;
    Command.header.stamp = now();
    Command.twist.linear.x = arVelocity.mLinear;
    Command.twist.angular.z = arVelocity.mAngular;

    mpVelocityPublisher->publish( Command );
}


//-----------------------------------------------------------------------------
// Reports the distances alongside the change, which shows why it happened.
// This is the main tool for tuning on the robot.
// Reports the latest wall distances whenever the active drive behaviour
// changes. This is for when checking and tuning the controller
//-----------------------------------------------------------------------------
void CWallFollowerNode::ReportBehaviourChange(
    const std::string& arPreviousBehaviour ) const
{
    const CWallDistances& Distances =
        mLidar.GetDistances();

    RCLCPP_INFO(
        get_logger(),
        "%s -> %s (front %.2f m, front-right gap %.2f m, right %.2f m)",
        arPreviousBehaviour.c_str(),
        mFollower.GetBehaviourName().c_str(),
        Distances.mFront,
        Distances.mFrontRightGap,
        Distances.mRight );
}
