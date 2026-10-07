//-----------------------------------------------------------------------------
// CSeekWall.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// Drives a right-hand arc: turning at speed / radius drives a circle of that
// radius.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CSeekWall.h"

//---Tuning--------------------------------------------------------------------
const double CSeekWall::kSpeed = 0.10;


//-----------------------------------------------------------------------------
CSeekWall::CSeekWall( double aTurnRadius )
    :
        CDriveBehaviour( "SeekWall" ),
        mTurnRadius( aTurnRadius )
{
}


//-----------------------------------------------------------------------------
CVelocity CSeekWall::GetVelocity( const CWallDistances& /* arDistances */ ) const
{
    // Negative turn rate: the arc is to the right, towards the lost wall
    const CVelocity Velocity = { kSpeed, -kSpeed / mTurnRadius };

    return Velocity;
}
