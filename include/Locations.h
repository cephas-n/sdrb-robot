/**
 * @file Locations.h
 * @author your name (you@domain.com)
 * @brief List of gps (latitude, longitude) coordinates of all locations
 * @version 0.1
 * @date 2023-05-04
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef _LOCATIONS_H_
#define _LOCATIONS_H_

const double allDestinations[4][2] {
    {35.1896057948247, 33.36170125562772}, // HOME
    {35.18953841962514, 33.36021290671352}, // location 1
    {35.18949230865301, 33.35948605869692}, // location 2
    {35.18946130019338, 33.358779607288376},  // location 3
};


/**
 * 11.a 
 * Variable where we'll copy allDestinations and replace the home locations by the current location
 * to make the calcullation of the path dynamic based on the current position
 * of the robot
 */
double final_destinations[NUM_OF_DESTINATIONS][2];

/**
 * Default destination (HOME)
 * 
 */
int active_destination = 0;
double current_destination[2]
{
  allDestinations[0][0],
  allDestinations[0][1]
};

#endif