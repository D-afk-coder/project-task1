//-----------------------------------------------------------------------------
// CFollowWall.h
//
// MTRX3760 Project 1, right wall follower.
//
// The behaviour for driving along the wall on the right: drive forward,
// steering to hold a set gap to the wall.
//-----------------------------------------------------------------------------

#ifndef CFOLLOWWALL_H
#define CFOLLOWWALL_H

#include "turtlebot3_gazebo/CDriveBehaviour.h"

//-----------------------------------------------------------------------------
// CFollowWall: drive forward, holding a set gap to the wall on the right.
//-----------------------------------------------------------------------------
class CFollowWall : public CDriveBehaviour
{
    public:
        //---Ctor---
        // aDesiredGap is the gap to hold to the wall [m]. A front-right gap
        // beyond aWallLostDist [m] means the diagonal ray is looking past the
        // end of the wall, so the reading straight to the right is steered on
        // instead.
        CFollowWall( double aDesiredGap, double aWallLostDist );

        //---Driving---
        virtual CVelocity GetVelocity( const CWallDistances& arDistances ) const;

    private:
        // TurnRateFor returns the turn rate that corrects a gap of aGap [m]
        double TurnRateFor( double aGap ) const;

        //---Tuning---
        static const double kSpeed;          // forward speed [m/s]
        static const double kSteeringGain;   // turn rate per metre of gap error [rad/s/m]
        static const double kMaxTurnRate;    // turn rate limit, either way [rad/s]

        //---Set by the wall follower---
        const double mDesiredGap;
        const double mWallLostDist;
};

#endif
