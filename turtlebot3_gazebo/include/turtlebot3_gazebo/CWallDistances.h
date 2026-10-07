//-----------------------------------------------------------------------------
// CWallDistances.h
//
// MTRX3760 Project 1, right wall follower.
//
// The three distances the wall follower steers by, as measured by CLidar from
// one laser scan. All are in metres, measured from the laser. When nothing is
// seen in a direction, its distance is the laser's maximum range.
//-----------------------------------------------------------------------------

#ifndef CWALLDISTANCES_H
#define CWALLDISTANCES_H

//-----------------------------------------------------------------------------
// CWallDistances: one scan's worth of distances, passed from the lidar to the
// wall follower and on to its behaviours.
//-----------------------------------------------------------------------------
struct CWallDistances
{
    double mFront;           // nearest obstacle ahead, across the robot's width
    double mFrontRightGap;   // gap to the right wall seen along the 45 degree
                             // diagonal; see CLidar.cpp for why it is used
    double mRight;           // nearest obstacle straight to the right
};

#endif
