//-----------------------------------------------------------------------------
// CLidar.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// Reduces laser scans to wall distances. See CLidar.h for what the class does.
// The look directions and widths are the tuning values for what the robot
// sees; they are defined at the top of this file.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CLidar.h"

#include <cmath>

//---Unit conversions----------------------------------------------------------
static const double kDegreesToRadians = M_PI / 180.0;
static const double kFullTurn = 2.0 * M_PI;   // [rad]

//---Where the lidar looks: { bearing, half width } in degrees------------------
// The front look is wide so that it covers the full width of the robot. The
// front-right look is narrow so that it behaves like a single diagonal ray.
// The right look is wide enough to still find the nearest point of the wall
// when the robot is not quite parallel to it.
const CLidar::Sector CLidar::kFront      = {   0.0, 30.0 };
const CLidar::Sector CLidar::kFrontRight = { -45.0,  3.0 };
const CLidar::Sector CLidar::kRight      = { -90.0, 10.0 };

const double CLidar::kScanTimeoutSec = 1.0;


//-----------------------------------------------------------------------------
CLidar::CLidar()
    :
        mDistances( { 0.0, 0.0, 0.0 } ),
        mLastScanTimeSec( 0.0 ),
        mHaveScan( false )
{
}


//-----------------------------------------------------------------------------
// The diagonal ray meets a wall on the right further along than the gap to it.
// Scaling its reading by the sine of its angle from straight ahead gives the
// gap back, taking the robot to be parallel to the wall. That gap grows both
// when the robot is further from the wall and when it points away from it, so
// it is a better thing to steer on than the reading straight to the right,
// which cannot tell which way the robot is pointing.
//-----------------------------------------------------------------------------
void CLidar::ReadScan( const sensor_msgs::msg::LaserScan& arScan, double aTimeSec )
{
    const bool Usable = !arScan.ranges.empty() && arScan.angle_increment > 0.0;

    if( Usable )
    {
        const double FrontRight = NearestInSector( arScan, kFrontRight );

        mDistances.mFront = NearestInSector( arScan, kFront );
        mDistances.mFrontRightGap =
            FrontRight * std::sin( -kFrontRight.mBearingDeg * kDegreesToRadians );
        mDistances.mRight = NearestInSector( arScan, kRight );

        mLastScanTimeSec = aTimeSec;
        mHaveScan = true;
    }
}


//-----------------------------------------------------------------------------
bool CLidar::HasFreshScan( double aNowSec ) const
{
    bool Fresh = false;

    if( mHaveScan )
    {
        Fresh = ( aNowSec - mLastScanTimeSec ) < kScanTimeoutSec;
    }

    return Fresh;
}


//-----------------------------------------------------------------------------
const CWallDistances& CLidar::GetDistances() const
{
    return mDistances;
}


//-----------------------------------------------------------------------------
// Readings outside the laser's valid range are skipped: Gazebo reports those
// as infinity, and a real laser may report them as zero. The caller has
// already checked that the scan has rays and a positive angle between them.
//-----------------------------------------------------------------------------
double CLidar::NearestInSector( const sensor_msgs::msg::LaserScan& arScan,
                                const Sector& arSector )
{
    double Nearest = arScan.range_max;

    const int RayCount = static_cast<int>( arScan.ranges.size() );

    // Angle of the sector centre measured from the first ray, within one turn
    double FromFirstRay = std::fmod( arSector.mBearingDeg * kDegreesToRadians - arScan.angle_min,
                                     kFullTurn );
    if( FromFirstRay < 0.0 )
    {
        FromFirstRay += kFullTurn;
    }

    const int CentreIndex =
        static_cast<int>( std::round( FromFirstRay / arScan.angle_increment ) );
    const int HalfWidth =
        static_cast<int>( std::round( arSector.mHalfWidthDeg * kDegreesToRadians /
                                      arScan.angle_increment ) );

    for( int Offset = -HalfWidth; Offset <= HalfWidth; ++Offset )
    {
        // The scan is a full circle, so stepping back past the first ray
        // carries on from the last one, and the other way round
        const int Index = ( ( CentreIndex + Offset ) % RayCount + RayCount ) % RayCount;
        const double Range = arScan.ranges[ static_cast<size_t>( Index ) ];

        const bool Valid = std::isfinite( Range ) &&
                           Range >= arScan.range_min &&
                           Range <= arScan.range_max;

        if( Valid && Range < Nearest )
        {
            Nearest = Range;
        }
    }

    return Nearest;
}
