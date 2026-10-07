//-----------------------------------------------------------------------------
// CFollowWall.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// Drives along the right wall with a proportional steering controller on the
// gap to the wall, limited to a maximum turn rate.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CFollowWall.h"

//---Tuning--------------------------------------------------------------------
const double CFollowWall::kSpeed = 0.15;
const double CFollowWall::kSteeringGain = 3.0;
const double CFollowWall::kMaxTurnRate = 0.8;


//-----------------------------------------------------------------------------
CFollowWall::CFollowWall( double aDesiredGap, double aWallLostDist )
    :
        CDriveBehaviour( "FollowWall" ),
        mDesiredGap( aDesiredGap ),
        mWallLostDist( aWallLostDist )
{
}


//-----------------------------------------------------------------------------
// Steers on the front-right gap whenever the diagonal ray can see the wall, so
// that the heading is corrected along with the gap and the robot settles
// instead of weaving. When that ray looks past the end of the wall, the
// reading straight to the right is used instead.
//-----------------------------------------------------------------------------
CVelocity CFollowWall::GetVelocity( const CWallDistances& arDistances ) const
{
    double Gap = arDistances.mRight;
    if( arDistances.mFrontRightGap < mWallLostDist )
    {
        Gap = arDistances.mFrontRightGap;
    }

    const CVelocity Velocity = { kSpeed, TurnRateFor( Gap ) };

    return Velocity;
}


//-----------------------------------------------------------------------------
double CFollowWall::TurnRateFor( double aGap ) const
{
    // Too far from the wall gives a positive error, and the wall is on the
    // right, so the correction is a turn to the right: a negative turn rate
    double TurnRate = -kSteeringGain * ( aGap - mDesiredGap );

    if( TurnRate > kMaxTurnRate )
    {
        TurnRate = kMaxTurnRate;
    }
    else if( TurnRate < -kMaxTurnRate )
    {
        TurnRate = -kMaxTurnRate;
    }

    return TurnRate;
}
