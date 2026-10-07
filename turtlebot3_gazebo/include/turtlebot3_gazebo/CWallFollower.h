//-----------------------------------------------------------------------------
// CWallFollower.h
//
// MTRX3760 Project 1, right wall follower.
//
// The decision-making for right wall following. Given the latest wall
// distances, CWallFollower decides which drive behaviour is in charge and
// returns the velocity that behaviour calls for:
//
//   TurnLeft    a wall ahead: turn left on the spot until the way is clear
//   SeekWall    no wall on the right: the wall has ended, arc right round it
//   FollowWall  otherwise: drive along the wall, holding a set gap to it
//   Stop        no up-to-date distances: stay still
//
// CWallFollower uses no ROS types, so it works the same in simulation and on
// the robot. It assumes the robot starts with a wall on its right.
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWER_H
#define CWALLFOLLOWER_H

#include "turtlebot3_gazebo/CFollowWall.h"
#include "turtlebot3_gazebo/CSeekWall.h"
#include "turtlebot3_gazebo/CStop.h"
#include "turtlebot3_gazebo/CTurnLeft.h"
#include "turtlebot3_gazebo/CVelocity.h"
#include "turtlebot3_gazebo/CWallDistances.h"

#include <string>

//-----------------------------------------------------------------------------
// CWallFollower: chooses a drive behaviour from the wall distances.
//-----------------------------------------------------------------------------
class CWallFollower
{
    public:
        //---Ctor---
        // Starts out stopped, until the first update with fresh distances.
        CWallFollower();

        // Not copyable: mpBehaviour points at one of this object's own
        // behaviours, so a copy would be steered by the original's.
        CWallFollower( const CWallFollower& ) = delete;
        CWallFollower& operator=( const CWallFollower& ) = delete;

        //---Following the wall---
        // Update chooses the behaviour for arDistances and returns the
        // velocity it calls for. aDistancesAreFresh is false when the
        // distances are out of date, and the robot then stops.
        CVelocity Update( const CWallDistances& arDistances, bool aDistancesAreFresh );

        //---Access---
        // GetBehaviourName names the behaviour chosen by the latest Update
        const std::string& GetBehaviourName() const;

    private:
        // ChooseBehaviour picks the behaviour for arDistances, most urgent
        // first, taking account of the behaviour currently in charge.
        const CDriveBehaviour* ChooseBehaviour( const CWallDistances& arDistances ) const;

        //---Decision distances [m], measured from the laser---
        static const double kDesiredWallDist;    // gap to hold to the right wall
        static const double kWallLostDist;       // right wall further than this: it has ended
        static const double kFrontBlockedDist;   // wall ahead nearer than this: turn
        static const double kFrontClearDist;     // once turning, turn until this is clear

        //---The behaviours, one of which is in charge---
        CFollowWall mFollowWall;
        CTurnLeft mTurnLeft;
        CSeekWall mSeekWall;
        CStop mStop;
        const CDriveBehaviour* mpBehaviour;   // the one in charge
};

#endif
