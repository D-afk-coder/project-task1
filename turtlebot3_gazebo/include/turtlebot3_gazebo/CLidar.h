//-----------------------------------------------------------------------------
// CLidar.h
//
// MTRX3760 Project 1, right wall follower.
//
// The robot's laser scanner, as the wall follower sees it. CLidar reduces each
// laser scan to the three distances in CWallDistances, and keeps track of
// whether scans are still arriving, so that the robot never drives on old
// measurements.
//
// CLidar is the only class that reads a laser scan message. Rays are looked up
// by the angles the scan reports, not by fixed index, so the same code works
// with the simulated laser (exactly 360 rays) and a real one (which need not
// be).
//-----------------------------------------------------------------------------

#ifndef CLIDAR_H
#define CLIDAR_H

#include "turtlebot3_gazebo/CWallDistances.h"

#include <sensor_msgs/msg/laser_scan.hpp>

//-----------------------------------------------------------------------------
// CLidar: turns laser scans into wall distances and reports when they stop.
//-----------------------------------------------------------------------------
class CLidar
{
    public:
        //---Ctor---
        CLidar();

        //---Reading scans---
        // ReadScan measures the wall distances in a newly arrived scan.
        // aTimeSec is when it arrived. A scan with no rays, or no angle
        // between them, cannot be searched; it is ignored, the distances keep
        // their previous values and the scan does not count as fresh.
        void ReadScan( const sensor_msgs::msg::LaserScan& arScan, double aTimeSec );

        // HasFreshScan is true while scans are still arriving: at least one
        // usable scan has been read, and the latest is younger than
        // kScanTimeoutSec at time aNowSec. When it is false the distances are
        // out of date and should not be driven on.
        bool HasFreshScan( double aNowSec ) const;

        //---Access to the latest measurements---
        const CWallDistances& GetDistances() const;

    private:
        //---Where the lidar looks---
        // A sector of the scan: a bearing in the laser frame, where 0 is
        // straight ahead and positive is anticlockwise (so the right hand side
        // is negative), and how far to look either side of it.
        struct Sector
        {
            double mBearingDeg;
            double mHalfWidthDeg;
        };

        static const Sector kFront;
        static const Sector kFrontRight;
        static const Sector kRight;

        // No usable scan for this long means the laser has stopped [s]
        static const double kScanTimeoutSec;

        // NearestInSector returns the nearest valid range in arSector of
        // arScan [m], or the laser's maximum range if nothing in the sector
        // gives a valid return.
        static double NearestInSector( const sensor_msgs::msg::LaserScan& arScan,
                                       const Sector& arSector );

        //---Latest measurements---
        CWallDistances mDistances;
        double mLastScanTimeSec;   // when the latest usable scan arrived [s]
        bool mHaveScan;            // false until the first usable scan
};

#endif
