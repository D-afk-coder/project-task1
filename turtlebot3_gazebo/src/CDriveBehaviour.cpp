//-----------------------------------------------------------------------------
// CDriveBehaviour.cpp
//
// MTRX3760 Project 1, right wall follower.
//
// The parts shared by every drive behaviour: its name. GetVelocity is pure
// virtual, so each derived behaviour supplies its own.
//-----------------------------------------------------------------------------

#include "turtlebot3_gazebo/CDriveBehaviour.h"


//-----------------------------------------------------------------------------
CDriveBehaviour::CDriveBehaviour( const std::string& arName )
    :
        mName( arName )
{
}


//-----------------------------------------------------------------------------
CDriveBehaviour::~CDriveBehaviour()
{
}


//-----------------------------------------------------------------------------
const std::string& CDriveBehaviour::GetName() const
{
    return mName;
}
