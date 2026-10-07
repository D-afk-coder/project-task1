//-----------------------------------------------------------------------------
// CStop.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// Stands still: zero speed and zero turn rate.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CStop.h"


//-----------------------------------------------------------------------------
CStop::CStop()
    :
        CDriveBehaviour( "Stop" )
{
}


//-----------------------------------------------------------------------------
CVelocity CStop::GetVelocity( const CWallDistances& /* arDistances */ ) const
{
    const CVelocity Velocity = { 0.0, 0.0 };

    return Velocity;
}
