//-----------------------------------------------------------------------------
// CVelocity.h
//
// MTRX3760 Project 1, right wall follower.
//
// A velocity for the robot to drive at, as chosen by a drive behaviour. It
// holds plain numbers so that the behaviours need no ROS types; the node turns
// it into a ROS velocity command.
//-----------------------------------------------------------------------------

#ifndef CVELOCITY_H
#define CVELOCITY_H

//-----------------------------------------------------------------------------
// CVelocity: a forward speed and a turn rate.
//-----------------------------------------------------------------------------
struct CVelocity
{
    double mLinear;    // forward speed [m/s]
    double mAngular;   // turn rate [rad/s]; positive turns left (anticlockwise)
};

#endif
