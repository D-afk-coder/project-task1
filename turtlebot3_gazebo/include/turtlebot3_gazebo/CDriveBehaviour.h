//-----------------------------------------------------------------------------
// CDriveBehaviour.h
//
// MTRX3760 Project 1, right wall follower.
//
// The base class for the wall follower's behaviours. Each behaviour is one way
// of driving (follow the wall, turn away from a wall ahead, round the end of a
// wall, stop) and works out its own velocity from the latest wall distances.
// The wall follower decides which behaviour is in charge and asks it for a
// velocity through this common interface, without needing to know which kind
// of behaviour it has.
//
// Adding a new way of driving means adding a new class derived from this one;
// nothing else that drives the robot needs to change.
//-----------------------------------------------------------------------------

#ifndef CDRIVEBEHAVIOUR_H
#define CDRIVEBEHAVIOUR_H

#include "turtlebot3_gazebo/CVelocity.h"
#include "turtlebot3_gazebo/CWallDistances.h"

#include <string>

//-----------------------------------------------------------------------------
// CDriveBehaviour: one way of driving the robot. Abstract.
//-----------------------------------------------------------------------------
class CDriveBehaviour
{
    public:
        //---Ctor/Dtor---
        // arName is a short name for the behaviour, used when reporting it
        explicit CDriveBehaviour( const std::string& arName );
        virtual ~CDriveBehaviour();

        //---Driving---
        // GetVelocity returns the velocity to drive at under this behaviour,
        // given the latest wall distances.
        virtual CVelocity GetVelocity( const CWallDistances& arDistances ) const = 0;

        //---Access---
        const std::string& GetName() const;

    private:
        const std::string mName;
};

#endif
