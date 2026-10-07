//-----------------------------------------------------------------------------
// CWallFollower.h
//
// MTRX3760 Project 1, right wall follower.
//
// The decision-making for right wall following. Given the latest wall
// distances, CWallFollower decides what the robot should be doing and returns
// the velocity for it:
//
//   TURN_LEFT      a wall ahead: turn left on the spot until the way is clear
//   SEEK_WALL      no wall on the right: the wall has ended, arc right round it
//   FOLLOW_WALL    otherwise: drive along the wall, holding a set gap to it
//   WAIT_FOR_SCAN  no up-to-date distances: stay still
//
// CWallFollower uses no ROS types, so it works the same in simulation and on
// the robot. It assumes the robot starts with a wall on its right.
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWER_H
#define CWALLFOLLOWER_H

#include "turtlebot3_gazebo/CVelocity.h"
#include "turtlebot3_gazebo/CWallDistances.h"

//-----------------------------------------------------------------------------
// CWallFollower: chooses what to do from the wall distances.
//-----------------------------------------------------------------------------
class CWallFollower
{
    public:
        //---Ctor---
        // Starts out waiting, until the first update with fresh distances.
        CWallFollower();

        //---Following the wall---
        // Update chooses what to do for arDistances and returns the velocity
        // for it. aDistancesAreFresh is false when the distances are out of
        // date, and the robot then stops.
        CVelocity Update( const CWallDistances& arDistances, bool aDistancesAreFresh );

        //---Access---
        // GetStateName names the state chosen by the latest Update
        const char* GetStateName() const;

    private:
        // What the robot is doing about the wall on its right
        enum eDriveState
        {
            WAIT_FOR_SCAN,
            FOLLOW_WALL,
            TURN_LEFT,
            SEEK_WALL
        };

        eDriveState ChooseState( const CWallDistances& arDistances ) const;
        double FollowWallTurnRate( const CWallDistances& arDistances ) const;

        //---Decision distances [m], measured from the laser---
        static const double kDesiredWallDist;    // gap to hold to the right wall
        static const double kWallLostDist;       // right wall further than this: it has ended
        static const double kFrontBlockedDist;   // wall ahead nearer than this: turn
        static const double kFrontClearDist;     // once turning, turn until this is clear

        //---Speeds [m/s] and turn rates [rad/s]---
        static const double kFollowSpeed;
        static const double kSeekSpeed;
        static const double kTurnRate;
        static const double kMaxFollowTurnRate;
        static const double kSteeringGain;       // [rad/s per m of gap error]

        eDriveState mState;
};

#endif
