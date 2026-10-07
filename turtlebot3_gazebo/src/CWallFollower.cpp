//-----------------------------------------------------------------------------
// CWallFollower.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// Chooses what the robot does and how fast it drives. The tuning values are
// defined at the top of this file. In narrow passages (under about 0.75 m)
// reduce all four decision distances together.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CWallFollower.h"

//---Decision distances [m]----------------------------------------------------
const double CWallFollower::kDesiredWallDist = 0.30;
const double CWallFollower::kWallLostDist = 0.60;
const double CWallFollower::kFrontBlockedDist = 0.32;
const double CWallFollower::kFrontClearDist = 0.50;

//---Speeds and turn rates-----------------------------------------------------
const double CWallFollower::kFollowSpeed = 0.15;
const double CWallFollower::kSeekSpeed = 0.10;
const double CWallFollower::kTurnRate = 0.6;
const double CWallFollower::kMaxFollowTurnRate = 0.8;
const double CWallFollower::kSteeringGain = 3.0;


//-----------------------------------------------------------------------------
CWallFollower::CWallFollower()
    :
        mState( WAIT_FOR_SCAN )
{
}


//-----------------------------------------------------------------------------
CVelocity CWallFollower::Update( const CWallDistances& arDistances, bool aDistancesAreFresh )
{
    CVelocity Velocity = { 0.0, 0.0 };

    if( aDistancesAreFresh )
    {
        mState = ChooseState( arDistances );
    }
    else
    {
        mState = WAIT_FOR_SCAN;
    }

    switch( mState )
    {
        case FOLLOW_WALL:
            Velocity.mLinear = kFollowSpeed;
            Velocity.mAngular = FollowWallTurnRate( arDistances );
            break;

        case TURN_LEFT:
            Velocity.mLinear = 0.0;
            Velocity.mAngular = kTurnRate;
            break;

        case SEEK_WALL:
            // Turning at speed / radius drives a circle of that radius. A
            // radius equal to the desired gap carries the robot round the end
            // of the wall it has just passed, at the gap it was following at.
            Velocity.mLinear = kSeekSpeed;
            Velocity.mAngular = -kSeekSpeed / kDesiredWallDist;
            break;

        case WAIT_FOR_SCAN:
        default:
            Velocity.mLinear = 0.0;
            Velocity.mAngular = 0.0;
            break;
    }

    return Velocity;
}


//-----------------------------------------------------------------------------
const char* CWallFollower::GetStateName() const
{
    const char* Name = "UNKNOWN";

    switch( mState )
    {
        case WAIT_FOR_SCAN:
            Name = "WAIT_FOR_SCAN";
            break;

        case FOLLOW_WALL:
            Name = "FOLLOW_WALL";
            break;

        case TURN_LEFT:
            Name = "TURN_LEFT";
            break;

        case SEEK_WALL:
            Name = "SEEK_WALL";
            break;

        default:
            break;
    }

    return Name;
}


//-----------------------------------------------------------------------------
// Two of the choices depend on the current state, so that the robot commits to
// a manoeuvre instead of flicking between two states:
//
//   Once turning away from a wall ahead, it keeps turning until the way is
//   well clear. Starting and stopping the turn at the same distance would
//   leave the robot sitting on the threshold.
//
//   Once rounding the end of a wall, it keeps going until the diagonal ray
//   picks up the wall's far side. The look straight to the right only catches
//   glimpses of the wall end on the way round, too little to steer by.
//-----------------------------------------------------------------------------
CWallFollower::eDriveState CWallFollower::ChooseState( const CWallDistances& arDistances ) const
{
    eDriveState NextState = FOLLOW_WALL;

    double BlockedDist = kFrontBlockedDist;
    if( mState == TURN_LEFT )
    {
        BlockedDist = kFrontClearDist;
    }

    const bool WallAheadRight = arDistances.mFrontRightGap < kWallLostDist;
    const bool WallOnRight = arDistances.mRight < kWallLostDist;

    const bool StillSeeking = ( mState == SEEK_WALL ) && !WallAheadRight;
    const bool WallGone = !WallAheadRight && !WallOnRight;

    if( arDistances.mFront < BlockedDist )
    {
        NextState = TURN_LEFT;
    }
    else if( WallGone || StillSeeking )
    {
        NextState = SEEK_WALL;
    }

    return NextState;
}


//-----------------------------------------------------------------------------
// Steers on the front-right gap whenever the diagonal ray can see the wall, so
// that the heading is corrected along with the gap and the robot settles
// instead of weaving. When that ray looks past the end of the wall, the
// reading straight to the right is used instead.
//-----------------------------------------------------------------------------
double CWallFollower::FollowWallTurnRate( const CWallDistances& arDistances ) const
{
    double Gap = arDistances.mRight;
    if( arDistances.mFrontRightGap < kWallLostDist )
    {
        Gap = arDistances.mFrontRightGap;
    }

    // Too far from the wall gives a positive error, and the wall is on the
    // right, so the correction is a turn to the right: a negative turn rate
    double TurnRate = -kSteeringGain * ( Gap - kDesiredWallDist );

    if( TurnRate > kMaxFollowTurnRate )
    {
        TurnRate = kMaxFollowTurnRate;
    }
    else if( TurnRate < -kMaxFollowTurnRate )
    {
        TurnRate = -kMaxFollowTurnRate;
    }

    return TurnRate;
}
