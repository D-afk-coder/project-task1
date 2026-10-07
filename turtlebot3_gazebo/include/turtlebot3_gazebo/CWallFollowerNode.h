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
// CWallFollowerNode.h
//
// MTRX3760 Project 1, right wall follower. Restructured from the ROBOTIS
// turtlebot3_drive node.
//
// The ROS node that makes a TurtleBot 3 follow the wall on its right. It is the
// only class that deals with ROS: it receives laser scans, runs the update
// loop and publishes velocity commands. The work in between is done by the
// objects it owns:
//
//   CLidar         reduces each laser scan to wall distances
//   CWallFollower  chooses a behaviour from those distances, and the velocity
//
// Subscribes to:  scan     (sensor_msgs/LaserScan)
// Publishes:      cmd_vel  (geometry_msgs/TwistStamped)
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWERNODE_H
#define CWALLFOLLOWERNODE_H

#include "turtlebot3_gazebo/CLidar.h"
#include "turtlebot3_gazebo/CVelocity.h"
#include "turtlebot3_gazebo/CWallFollower.h"

#include <chrono>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

//-----------------------------------------------------------------------------
// CWallFollowerNode: connects the wall follower to the robot through ROS.
//-----------------------------------------------------------------------------
class CWallFollowerNode : public rclcpp::Node
{
    public:
        //---Ctor/Dtor---
        CWallFollowerNode();
        ~CWallFollowerNode();

    private:
        //---ROS callbacks---
        // OnScan hands each arriving laser scan to the lidar
        void OnScan( const sensor_msgs::msg::LaserScan::SharedPtr apScan );

        // OnUpdateTimer runs the wall follower on the latest distances and
        // sends the velocity it chooses, at a steady rate
        void OnUpdateTimer();

        //---Helpers---
        void PublishVelocity( const CVelocity& arVelocity );
        void ReportBehaviourChange( const std::string& arPreviousBehaviour ) const;

        //---Settings---
        static const std::string kNodeName;
        static const std::string kScanTopic;
        static const std::string kVelocityTopic;
        static const int kVelocityQueueDepth;
        static const std::chrono::milliseconds kUpdatePeriod;
        static const int kStaleWarningPeriodMs;   // how often to repeat the warning

        //---The wall follower---
        CLidar mLidar;
        CWallFollower mFollower;

        //---ROS connections---
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mpVelocityPublisher;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mpScanSubscription;
        rclcpp::TimerBase::SharedPtr mpUpdateTimer;
};

#endif
