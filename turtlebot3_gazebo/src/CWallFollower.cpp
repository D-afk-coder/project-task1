//-----------------------------------------------------------------------------
// CWallFollower.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// Chooses which drive behaviour is in charge. The decision distances are the
// tuning values for when the robot changes what it is doing; they are defined
// at the top of this file. In narrow passages (under about 0.75 m) reduce all
// four together.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CWallFollower.h"

//---Decision distances [m]----------------------------------------------------
const double CWallFollower::kDesiredWallDist = 0.30;
const double CWallFollower::kWallLostDist = 0.60;
const double CWallFollower::kFrontBlockedDist = 0.32;
const double CWallFollower::kFrontClearDist = 0.50;


//-----------------------------------------------------------------------------
// Following holds kDesiredWallDist, and rounding the end of a wall uses it as
// the arc radius, so the robot keeps the same gap all the way round.
//-----------------------------------------------------------------------------
CWallFollower::CWallFollower()
    :
        mFollowWall( kDesiredWallDist, kWallLostDist ),
        mTurnLeft(),
        mSeekWall( kDesiredWallDist ),
        mStop(),
        mpBehaviour( &mStop )
{
}


//-----------------------------------------------------------------------------
CVelocity CWallFollower::Update( const CWallDistances& arDistances, bool aDistancesAreFresh )
{
    if( aDistancesAreFresh )
    {
        mpBehaviour = ChooseBehaviour( arDistances );
    }
    else
    {
        mpBehaviour = &mStop;
    }

    return mpBehaviour->GetVelocity( arDistances );
}


//-----------------------------------------------------------------------------
const std::string& CWallFollower::GetBehaviourName() const
{
    return mpBehaviour->GetName();
}


//-----------------------------------------------------------------------------
// Two of the choices depend on what the robot is already doing, so that it
// commits to a manoeuvre instead of flicking between two behaviours:
//
//   Once turning away from a wall ahead, it keeps turning until the way is
//   well clear. Starting and stopping the turn at the same distance would
//   leave the robot sitting on the threshold.
//
//   Once rounding the end of a wall, it keeps going until the diagonal ray
//   picks up the wall's far side. The look straight to the right only catches
//   glimpses of the wall end on the way round, too little to steer by.
//-----------------------------------------------------------------------------
const CDriveBehaviour* CWallFollower::ChooseBehaviour( const CWallDistances& arDistances ) const
{
    const CDriveBehaviour* pNext = &mFollowWall;

    double BlockedDist = kFrontBlockedDist;
    if( mpBehaviour == &mTurnLeft )
    {
        BlockedDist = kFrontClearDist;
    }

    const bool WallAheadRight = arDistances.mFrontRightGap < kWallLostDist;
    const bool WallOnRight = arDistances.mRight < kWallLostDist;

    const bool StillSeeking = ( mpBehaviour == &mSeekWall ) && !WallAheadRight;
    const bool WallGone = !WallAheadRight && !WallOnRight;

    if( arDistances.mFront < BlockedDist )
    {
        pNext = &mTurnLeft;
    }
    else if( WallGone || StillSeeking )
    {
        pNext = &mSeekWall;
    }

    return pNext;
}
