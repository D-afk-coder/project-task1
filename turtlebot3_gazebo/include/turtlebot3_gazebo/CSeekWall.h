//-----------------------------------------------------------------------------
// CSeekWall.h
//
// MTRX3760 Project 1, right wall follower.
//
// The behaviour for when the wall on the right has ended: arc right, round the
// end of the wall, until the wall's far side comes back into view.
//-----------------------------------------------------------------------------

#ifndef CSEEKWALL_H
#define CSEEKWALL_H

#include "turtlebot3_gazebo/CDriveBehaviour.h"

//-----------------------------------------------------------------------------
// CSeekWall: drive a right-hand arc of a set radius.
//-----------------------------------------------------------------------------
class CSeekWall : public CDriveBehaviour
{
    public:
        //---Ctor---
        // aTurnRadius is the radius of the arc [m]. Setting it to the gap the
        // robot was following the wall at carries the robot round the end of
        // the wall at that same gap.
        explicit CSeekWall( double aTurnRadius );

        //---Driving---
        // The arc is the same whatever the distances; the wall follower
        // decides when the wall has been found again.
        virtual CVelocity GetVelocity( const CWallDistances& arDistances ) const;

    private:
        //---Tuning---
        static const double kSpeed;   // forward speed round the arc [m/s]

        //---Set by the wall follower---
        const double mTurnRadius;
};

#endif
