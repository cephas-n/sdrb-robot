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
    {35.222715254543246, 33.41966041944938}, // HOME
    {35.22268618632446, 33.41881367324544}, // location 1
    {35.22402826536914, 33.41790091301004},  // location 2
    {35.221672450703295, 33.417513793983275}, // location 3
};


/**
 * 11.a 
 * Variable where we'll copy allDestinations and replace the home locations by the current location
 * to make the calcullation of the path dynamic based on the current position
 * of the robot
 */
double final_destinations[NUM_OF_DESTINATIONS][2];


#endif