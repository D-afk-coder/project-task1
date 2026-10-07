//-----------------------------------------------------------------------------
// CTurnLeft.h
//
// MTRX3760 Project 1, right wall follower.
//
// The behaviour for a wall ahead: turn left on the spot, away from it, keeping
// the wall that was ahead on the right.
//-----------------------------------------------------------------------------

#ifndef CTURNLEFT_H
#define CTURNLEFT_H

#include "turtlebot3_gazebo/CDriveBehaviour.h"

//-----------------------------------------------------------------------------
// CTurnLeft: turn left on the spot.
//-----------------------------------------------------------------------------
class CTurnLeft : public CDriveBehaviour
{
    public:
        //---Ctor---
        CTurnLeft();

        //---Driving---
        // The turn is the same whatever the distances; the wall follower
        // decides when it has turned far enough.
        virtual CVelocity GetVelocity( const CWallDistances& arDistances ) const;

    private:
        //---Tuning---
        static const double kTurnRate;   // [rad/s], anticlockwise
};

#endif
