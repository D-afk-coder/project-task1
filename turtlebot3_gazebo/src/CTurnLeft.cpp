//-----------------------------------------------------------------------------
// CTurnLeft.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// Turns left on the spot at a fixed rate.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CTurnLeft.h"

//---Tuning--------------------------------------------------------------------
const double CTurnLeft::kTurnRate = 0.6;


//-----------------------------------------------------------------------------
CTurnLeft::CTurnLeft()
    :
        CDriveBehaviour( "TurnLeft" )
{
}


//-----------------------------------------------------------------------------
CVelocity CTurnLeft::GetVelocity( const CWallDistances& /* arDistances */ ) const
{
    const CVelocity Velocity = { 0.0, kTurnRate };

    return Velocity;
}
