//-----------------------------------------------------------------------------
// CStop.h
//
// MTRX3760 Project 1, right wall follower.
//
// The behaviour for when there are no up-to-date wall distances to drive on,
// because no laser scan has arrived yet or scans have stopped arriving: stay
// still.
//-----------------------------------------------------------------------------

#ifndef CSTOP_H
#define CSTOP_H

#include "turtlebot3_gazebo/CDriveBehaviour.h"

//-----------------------------------------------------------------------------
// CStop: stand still.
//-----------------------------------------------------------------------------
class CStop : public CDriveBehaviour
{
    public:
        //---Ctor---
        CStop();

        //---Driving---
        virtual CVelocity GetVelocity( const CWallDistances& arDistances ) const;
};

#endif
